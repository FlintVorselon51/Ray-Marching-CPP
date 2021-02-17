#include "cuda_runtime.h"
#include <cuda.h>
#include <math.h>
#include "Point.h"
#include "Camera.h"
#include <iostream>
#include <SDL.h>

__global__ void kernel(unsigned char* pixels, int* int_data, float* float_data, float* objects) {

    const int blockId = blockIdx.x + blockIdx.y * gridDim.x;
    const int threadId = blockId * (blockDim.x * blockDim.y) + (threadIdx.y * blockDim.x) + threadIdx.x;

    const int max_grid = int_data[0];
    const int max_block = int_data[1];
    const int texture_height = int_data[2];

    const int x = max_grid * blockIdx.y + blockIdx.x;
    const int y = max_block * threadIdx.y + threadIdx.x;

    float end_point_x = float_data[0] + float_data[3] * x;
    float end_point_y = float_data[1] + float_data[4] * y;
    float end_point_z = float_data[2] + float_data[5] * x;

    const float to_camera_x = end_point_x;
    const float to_camera_y = end_point_y;
    const float to_camera_z = end_point_z;

    float camera_x = float_data[6];
    float camera_y = float_data[7];
    float camera_z = float_data[8];

    // !!!
    const int number_of_objects = 1;

    float length;
    float coff;

    float min_distance;
    float distance;
    int idx;

    for (int i = 0; i < 20; i++) {
        min_distance = 0;
        distance = 0;
        idx = 0;
        for (int j = 0; j < 1; j++) {
            if (objects[j * 10] == 1) {
                distance = sqrtf(
                    powf(objects[j * 10 + 1] - camera_x, 2) +
                    powf(objects[j * 10 + 2] - camera_y, 2) +
                    powf(objects[j * 10 + 3] - camera_z, 2)); // - радиус круга
                distance -= objects[j * 10 + 7];
            }
            if (distance < min_distance || j == 0) {
                min_distance = distance;
                idx = j;
            }
            if (min_distance < 0.01) {
                if (objects[j * 10] == 1) {
                    pixels[(y * texture_height + x) * 4] = objects[idx * 10 + 6];
                    pixels[(y * texture_height + x) * 4 + 1] = objects[idx * 10 + 5];
                    pixels[(y * texture_height + x) * 4 + 2] = objects[idx * 10 + 4];
                    return;
                }
            }
            else if (min_distance > 20) {
                return;
            }
        }

        end_point_x = camera_x + to_camera_x;
        end_point_y = camera_y + to_camera_y;
        end_point_z = camera_z + to_camera_z;

        length = sqrtf(
            powf(end_point_x - camera_x, 2) +
            powf(end_point_y - camera_y, 2) +
            powf(end_point_z - camera_z, 2));
        coff = min_distance / length;

        camera_x = camera_x + (end_point_x - camera_x) * coff;
        camera_y = camera_y + (end_point_y - camera_y) * coff;
        camera_z = camera_z + (end_point_z - camera_z) * coff;
    }
}

int main(int argc, char** argv) {

    const unsigned int screen_width = 512;
    const unsigned int screen_height = 512;

    const unsigned int texture_width = 512;
    const unsigned int texture_height = 512;

    const int max_grid = ceil(sqrt(texture_width));
    const int max_block = ceil(sqrt(texture_height));

    dim3 threadsPerBlock(max_block, max_block);
    dim3 numBlocks(max_grid, max_grid);

    Point* camera_coordinates = new Point();
    Camera* camera = new Camera();

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_Window* window = SDL_CreateWindow
    (
        "SDL2",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        screen_width, screen_height,
        SDL_WINDOW_SHOWN
    );

    SDL_Renderer* renderer = SDL_CreateRenderer
    (
        window,
        -1,
        SDL_RENDERER_ACCELERATED
    );

    SDL_RendererInfo info;
    SDL_GetRendererInfo(renderer, &info);

    SDL_Texture* texture = SDL_CreateTexture
    (
        renderer,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        texture_width, texture_height
    );

    // int* pixels = new int[texture_width * texture_height * 4]{ 100 };
    // CUDA

    size_t size = (texture_width * texture_height * 4) * sizeof(unsigned char);
    size_t int_data_size = 3 * sizeof(int);
    size_t float_data_size = 10 * sizeof(float);
    size_t objects_size = 10 * 1 * sizeof(float);

    unsigned char* pixels = (unsigned char*)malloc(size);
    for (int i = 0; i < (texture_width * texture_height * 4); i++) {
        pixels[i] = 0;
    }

    int* int_data = (int*)malloc(int_data_size);
    int_data[0] = max_grid;
    int_data[1] = max_block;
    int_data[2] = texture_height;

    float* float_data = (float*)malloc(float_data_size);
    camera->update();
    float_data[0] = camera->get_start_x();
    float_data[1] = camera->get_start_y();
    float_data[2] = camera->get_start_z();
    float_data[3] = camera->get_delta_x() / texture_width;
    float_data[4] = camera->get_delta_y() / texture_height;
    float_data[5] = camera->get_delta_z() / texture_width;
    float_data[6] = camera_coordinates->get_x();
    float_data[7] = camera_coordinates->get_y();
    float_data[8] = camera_coordinates->get_z();

    float* objects = (float*)malloc(objects_size);
    objects[0] = 1;
    objects[1] = 5;
    objects[2] = 0;
    objects[3] = 0;
    objects[4] = 255;
    objects[5] = 0;
    objects[6] = 0;
    objects[7] = 3;
    objects[8] = 0;
    objects[9] = 0;

    unsigned char* d_pixels;
    int* d_int_data;
    float* d_float_data;
    float* d_objects;

    cudaMalloc(&d_pixels, size);
    cudaMalloc(&d_int_data, int_data_size);
    cudaMalloc(&d_float_data, float_data_size);
    cudaMalloc(&d_objects, objects_size);

    cudaMemcpy(d_pixels, pixels, size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_int_data, int_data, int_data_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_float_data, float_data, float_data_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_objects, objects, objects_size, cudaMemcpyHostToDevice);

    SDL_Event event;
    bool running = true;

    unsigned int frames = 0;
    Uint64 start = SDL_GetPerformanceCounter();

    while (running) {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);

        while (SDL_PollEvent(&event))
        {
            if ((SDL_QUIT == event.type) || (SDL_KEYDOWN == event.type && SDL_SCANCODE_ESCAPE == event.key.keysym.scancode))
            {
                running = false;
                break;
            }
        }

        cudaMemcpy(d_int_data, int_data, int_data_size, cudaMemcpyHostToDevice);
        cudaMemcpy(d_float_data, float_data, float_data_size, cudaMemcpyHostToDevice);
        cudaMemcpy(d_objects, objects, objects_size, cudaMemcpyHostToDevice);

        kernel <<<numBlocks, threadsPerBlock>>> (d_pixels, d_int_data, d_float_data, d_objects);

        cudaMemcpy(pixels, d_pixels, size, cudaMemcpyDeviceToHost);

        SDL_UpdateTexture
        (
            texture,
            NULL,
            pixels,
            texture_width * 4
        );

        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        frames++;
        const Uint64 end = SDL_GetPerformanceCounter();
        const static Uint64 freq = SDL_GetPerformanceFrequency();
        const double seconds = (end - start) / static_cast<double>(freq);
        if (seconds > 2.0)
        {
            std::cout
                << frames << " frames in "
                << std::fixed << seconds << " seconds = "
                << std::fixed << frames / seconds << " FPS ("
                << std::fixed << (seconds * 1000.0) / frames << " ms/frame)"
                << std::endl;
            start = end;
            frames = 0;
        }
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    cudaFree(d_pixels);
    cudaFree(d_int_data);
    cudaFree(d_float_data);

    return 0;
}
