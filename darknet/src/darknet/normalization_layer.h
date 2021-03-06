#ifndef NORMALIZATION_LAYER_H
#define NORMALIZATION_LAYER_H

#include "darknet/image.h"
#include "darknet/layer.h"
#include "darknet/network.h"

layer make_normalization_layer(int batch, int w, int h, int c, int size, float alpha, float beta, float kappa);
void resize_normalization_layer(layer *layer, int h, int w);
void forward_normalization_layer(const layer layer, network_state state);
void backward_normalization_layer(const layer layer, network_state state);
void visualize_normalization_layer(layer layer, char *window);

#ifdef GPU
void forward_normalization_layer_gpu(const layer layer, network_state state);
void backward_normalization_layer_gpu(const layer layer, network_state state);
#endif

#endif
