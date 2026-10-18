// Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
// SPDX-License-Identifier: Apache-2.0

attribute vec4 aPosition;
attribute vec4 aTextureCoord;
varying vec2 vTextureCoord;
uniform mat4 uMVPMatrix;
void main() {
  gl_Position = uMVPMatrix * aPosition;
  vTextureCoord = aTextureCoord.xy;
}