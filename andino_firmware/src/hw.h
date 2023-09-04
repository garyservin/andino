// BSD 3-Clause License
//
// Copyright (c) 2023, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

// GPIO35 (pin ), LEFT ENCODER PIN A
#define LEFT_ENCODER_A_GPIO_PIN 27

// GPIO34 (pin ), LEFT ENCODER PIN B
#define LEFT_ENCODER_B_GPIO_PIN 14

// GPIO33 (pin ), LEFT MOTOR DRIVER PWM PIN
#define LEFT_MOTOR_PWM_GPIO_PIN 26

// GPIO32 (pin ), LEFT MOTOR DRIVER DIRECTION PIN
#define LEFT_MOTOR_DIRECTION_GPIO_PIN 25


// GPIO14 (pin ), RIGHT ENCODER PIN A
#define RIGHT_ENCODER_A_GPIO_PIN 35

// GPIO27 (pin ), RIGHT ENCODER PIN B
#define RIGHT_ENCODER_B_GPIO_PIN 34

// GPIO26 (pin ), RIGHT MOTOR DRIVER PWM PIN
#define RIGHT_MOTOR_PWM_GPIO_PIN 33

// GPIO25 (pin ), RIGHT MOTOR DRIVER DIRECTION PIN
#define RIGHT_MOTOR_DIRECTION_GPIO_PIN 32

#define ROTARY_ENCODER_STEPS 1

#define MAX_VALUE_ENCODERS 2147483647