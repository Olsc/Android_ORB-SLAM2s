// Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
// SPDX-License-Identifier: Apache-2.0

precision mediump float;
varying vec2 vTextureCoord;
uniform sampler2D sTexture;
void main() {
    gl_FragColor=texture2D(sTexture, vTextureCoord);
}