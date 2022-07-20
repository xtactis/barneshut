#include "raylib.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <algorithm>
#include <vector>
#include <random>

#define N_POINTS 100000
#define THETA 1.f
#define G_CONSTANT 1
#define DT 0.001

enum NodeType {
    EMPTY = 0,
    LEAF,
    PARENT
};

struct MemoryPool {
    char *data;
    unsigned long long ptr = 0;
    const unsigned long long total;

    MemoryPool() : total(64*1024*1024) {
        //printf("total mem: %10llu bytes\n", total);
        data = (char *)malloc(total);
    }

    char *alloc(unsigned long long amount) {
        static int alloc_count = 0;
        alloc_count += 1;
        if (ptr+amount > total) {
            printf("%llu + %llu > %llu too many nodes (%d)!!!!! crashing lol\n", ptr, amount, total, alloc_count);
            exit(-1);
        }
        ptr += amount;
        return data+ptr-amount;
    }
};

struct Point {
    float x, y;
    float vel_x, vel_y;
    float mass;
};

struct Force {
    float x, y;

    void normalize() {
        float hyp = sqrt(x*x+y*y);
        if (hyp == 0) {
            //printf("oh fuck\n");
        }
        x /= hyp;
        y /= hyp;
    }
};

static inline float dist2D(float x1, float y1, float x2, float y2) {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) + 0.0025;
}

MemoryPool mem;
struct QuadTree {
    float pos_x, pos_y;
    float size;
    NodeType type;
    static int calc;

    float el_x, el_y;
    QuadTree *children[4];

    float mass_x=0, mass_y=0;
    float mass=0;

    QuadTree(float top, float bottom, float left, float right) 
        : pos_x(left), pos_y(top), size(std::max(right-left, bottom-top)), type(EMPTY) {}
    QuadTree() : pos_x(-40), pos_y(-40), size(80), type(EMPTY) {}
    void reset_node(float x=0.0, float y=0.0, float sz=1.0) {
        pos_x = x;
        pos_y = y;
        size = sz;
        mass = 0;
        mass_x = 0; mass_y = 0;
        this->type = EMPTY;
    }

    int child_index(float x, float y) {
        return ((x > pos_x+size/2) ? 1 : 0) + 
               ((y > pos_y+size/2) ? 2 : 0);
    }

    void add(float x, float y, float m) {
        //printf("%d %f\n", type, m);
        if (type == EMPTY) {
            el_x = x;
            el_y = y;
            mass = m;
            type = LEAF;
        } else if (type == LEAF) {
            if (x == el_x && y == el_y) return;
            float half_size = size/2;
            for (int i = 0; i < 4; ++i) {
                children[i] = (QuadTree *)mem.alloc(sizeof(QuadTree));
            }
            children[0]->reset_node(pos_x, pos_y, half_size);
            children[1]->reset_node(pos_x+half_size, pos_y, half_size);
            children[2]->reset_node(pos_x, pos_y+half_size, half_size);
            children[3]->reset_node(pos_x+half_size, pos_y+half_size, half_size);

            children[child_index(x, y)]->add(x, y, m);
            children[child_index(el_x, el_y)]->add(el_x, el_y, mass);

            mass += m;
            type = PARENT;
        } else if (type == PARENT) {
            mass += m;
            children[child_index(x, y)]->add(x, y, m);
        } else {
            //printf("wyd\n");
        }
    }

    int depth() {
        if (type == EMPTY) {
            return 0;
        } else if (type == LEAF) {
            return 1;
        } else { // type == PARENT
            int m = 0;
            for (int i = 0; i < 4; ++i) {
                m = std::max(m, children[i]->depth());
            }
            return m+1;
        }
    }

    void calculate_centers_of_mass() {
        calc = 0;
        if (type == EMPTY) {
            mass = 0;
            mass_x = 0; // doesn't matter
            mass_y = 0; // doesn't matter
        } else if (type == LEAF) {
            mass_x = el_x;
            mass_y = el_y;
        } else { // type == PARENT
            for (int i = 0; i < 4; ++i) {
                children[i]->calculate_centers_of_mass();
                mass_x += children[i]->mass_x * children[i]->mass;
                mass_y += children[i]->mass_y * children[i]->mass;
            }
            mass_x /= mass;
            mass_y /= mass;
        }
    }

    Force force_at(float x, float y, float m) {
        if (type == EMPTY) {
            return {0, 0};
        } else if (type == LEAF) {
            ++calc;
            const float dist = dist2D(x, y, el_x, el_y);
            Force force = {el_x-x, el_y-y};
            //force.normalize();
            float magnitude = G_CONSTANT*mass/(dist*dist*dist);
            force.x *= magnitude;
            force.y *= magnitude;
            return force;
        } else { // type == PARENT
            const float dist = dist2D(x, y, mass_x, mass_y);
            if (size / dist <= THETA) {
                ++calc;
                Force force = {mass_x-x, mass_y-y};
                //force.normalize();
                float magnitude = G_CONSTANT*mass/(dist*dist*dist);
                force.x *= magnitude;
                force.y *= magnitude;
                return force;
            } else {
                Force force = {0, 0};
                for (int i = 0; i < 4; ++i) {
                    if (children[i]->type == EMPTY) continue;
                    if (children[i]->type == LEAF && children[i]->el_x == x && children[i]->el_y == y) continue;
                    Force tmp = children[i]->force_at(x, y, m);
                    force.x += tmp.x;
                    force.y += tmp.y;
                }
                return force;
            }
        }
    }
};
int QuadTree::calc = 0;

inline float rand_01() {
    static std::default_random_engine generator;
    static std::uniform_real_distribution<float> distribution(0, 1.0);
    return distribution(generator);
}

int main() {
    srand(42);
    const int screenWidth = 800;
    const int screenHeight = 800;
    InitWindow(screenWidth, screenHeight, "barnes hut");
    SetTargetFPS(30);

    float top = 0, bottom = 0, left = 0, right = 0;

    std::vector<Point> points;
    points.reserve(N_POINTS);

    points.push_back({0, 0, 0, 0, 200000});
    for (int i = 1; i < N_POINTS; ++i) {
        float r = 10.5 * rand_01() + 1.5;
        float theta = 2 * PI * rand_01();
        float x = r*cos(theta);
        float y = r*sin(theta);
        float M = G_CONSTANT*200000;
        float v = sqrt(M/abs(r));
        top = std::min(top, y);
        bottom = std::max(bottom, y);
        left = std::min(left, x);
        right = std::max(right, x);
        points.push_back({x, y, -v*sin(theta), v*cos(theta), 1});
    }

    int frame_number = 0;
    while (!WindowShouldClose() && frame_number < 10) {
        QuadTree qt(top, bottom, left, right);
        for (auto &point: points) {
            //printf("%lf %lf %lf %lf %lf\n", point.x, point.y, point.vel_x, point.vel_y, point.mass);
            qt.add(point.x, point.y, point.mass);
        }

        BeginDrawing();
            ClearBackground(BLACK);

            for (const auto &point: points) {
                DrawPixel(point.x*screenWidth/32 + screenWidth/2, point.y*screenHeight/32 + screenHeight/2, point.mass > 1000 ? RED : Color{255, 255, 255, 90}); 
            }

            DrawText(TextFormat("calculations: %d", qt.calc), 10, 10, 20, WHITE);
            DrawText(TextFormat("theta: %.5lf", THETA), 10, 35, 20, WHITE);
        EndDrawing();

        //ExportImage(LoadImageFromScreen(), TextFormat("frames/frame%03d.png", frame_number));
        ++frame_number;

        qt.calculate_centers_of_mass();
        //printf("%lf %lf %lf %d\n", qt.mass, qt.mass_x, qt.mass_y, qt.depth());
        top = 0, bottom = 0, left = 0, right = 0;
        for (auto &point: points) {
            //if (point.mass > 10) continue;
            const Force force = qt.force_at(point.x, point.y, point.mass);
            //printf("%lf %lf | ", force.x, force.y);
            point.vel_x += DT*force.x;
            point.vel_y += DT*force.y;
            point.x += DT*point.vel_x;
            point.y += DT*point.vel_y;

            top = std::min(top, point.y);
            bottom = std::max(bottom, point.y);
            left = std::min(left, point.x);
            right = std::max(right, point.x);
        }
        //printf("\n");
        mem.ptr = 0;
    }

    CloseWindow();

    return 0;
}