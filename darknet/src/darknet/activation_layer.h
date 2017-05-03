#ifndef ACTIVATION_LAYER_H
#define ACTIVATION_LAYER_H

#include "darknet/activations.h"
#include "darknet/layer.h"
#include "darknet/network.h"

layer make_activation_layer(int batch, int inputs, ACTIVATION activation);

void forward_activation_layer(layer l, network_state state);
void backward_activation_layer(layer l, network_state state);

#ifdef GPU
void forward_activation_layer_gpu(layer l, network_state state);
void backward_activation_layer_gpu(layer l, network_state state);
#endif

#endif

