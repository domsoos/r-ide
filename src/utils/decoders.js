// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/
//
// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright 2018-2021 Cruise LLC
//
//   This source code is licensed under the Apache License, Version 2.0,
//   found at http://www.apache.org/licenses/LICENSE-2.0
//   You may not use this file except in compliance with the License.
//
//  This file is a modified version of the original file


export function decodeBGRA8(rgba, width, height, output){
    let inIdx = 0;
    let outIdx = 0;

    for (let i = 0; i < width * height; i++) {
        const b = rgba[inIdx++];
        const g = rgba[inIdx++];
        const r = rgba[inIdx++];
        const a = rgba[inIdx++];

        output[outIdx++] = r;
        output[outIdx++] = g;
        output[outIdx++] = b;
        output[outIdx++] = a;
    }
}

function yuvToRGBA8(y1, u, y2, v, c, output){
    // rgba
    output[c] = y1 + 1.402 * v;
    output[c + 1] = y1 - 0.34414 * u - 0.71414 * v;
    output[c + 2] = y1 + 1.772 * u;
    output[c + 3] = 255;
  
    // rgba
    output[c + 4] = y2 + 1.402 * v;
    output[c + 5] = y2 - 0.34414 * u - 0.71414 * v;
    output[c + 6] = y2 + 1.772 * u;
    output[c + 7] = 255;
  }

export function decodeYUV(yuv, width, height, output){
    let c = 0;
    let off = 0;
  
    // populate 2 pixels at a time
    const max = height * width;
    for (let r = 0; r <= max; r += 2) {
      const u = yuv[off] - 128;
      const y1 = yuv[off + 1];
      const v = yuv[off + 2] - 128;
      const y2 = yuv[off + 3];
      yuvToRGBA8(y1, u, y2, v, c, output);
      c += 8;
      off += 4;
    }
}

export function decodeYUYV(yuyv, width, height, output){
    let c = 0;
    let off = 0;
  
    // populate 2 pixels at a time
    const max = height * width;
    for (let r = 0; r <= max; r += 2) {
      const y1 = yuyv[off];
      const u = yuyv[off + 1] - 128;
      const y2 = yuyv[off + 2];
      const v = yuyv[off + 3] - 128;
      yuvToRGBA8(y1, u, y2, v, c, output);
      c += 8;
      off += 4;
    }
}

export function decodeRGB8(rgb, width, height, output){
    let inIdx = 0;
    let outIdx = 0;
  
    for (let i = 0; i < width * height; i++) {
      const r = rgb[inIdx++];
      const g = rgb[inIdx++];
      const b = rgb[inIdx++];

      output[outIdx++] = r;
      output[outIdx++] = g;
      output[outIdx++] = b;
      output[outIdx++] = 255;
    }
}

export function decodeRGBA8(rgba, width, height, output){
    let inIdx = 0;
    let outIdx = 0;
  
    for (let i = 0; i < width * height; i++) {
      const r = rgba[inIdx++];
      const g = rgba[inIdx++];
      const b = rgba[inIdx++];
      const a = rgba[inIdx++];
  
      output[outIdx++] = r;
      output[outIdx++] = g;
      output[outIdx++] = b;
      output[outIdx++] = a;
    }
}

export function decodeBGR8(bgr, width, height, output){
    let inIdx = 0;
    let outIdx = 0;
  
    for (let i = 0; i < width * height; i++) {
      const b = bgr[inIdx++];
      const g = bgr[inIdx++];
      const r = bgr[inIdx++];
  
      output[outIdx++] = r;
      output[outIdx++] = g;
      output[outIdx++] = b;
      output[outIdx++] = 255;
    }
}


export function decodeFloat1c(gray, width, height, isBigendian, output){
    const view = new DataView(gray.buffer, gray.byteOffset);
  
    let outIdx = 0;
    for (let i = 0; i < width * height * 4; i += 4) {
      const val = view.getFloat32(i, !isBigendian) * 255;
      output[outIdx++] = val;
      output[outIdx++] = val;
      output[outIdx++] = val;
      output[outIdx++] = 255;
    }
}

export function decodeMono8(mono8, width, height, output){
    let inIdx = 0;
    let outIdx = 0;
  
    for (let i = 0; i < width * height; i++) {
      const ch = mono8[inIdx++];
      output[outIdx++] = ch;
      output[outIdx++] = ch;
      output[outIdx++] = ch;
      output[outIdx++] = 255;
    }
}

export function decodeMono16(mono16, width, height, isBigendian, output, options){ //options?: { minValue?: number; maxValue?: number }
    const view = new DataView(mono16.buffer, mono16.byteOffset);

    // Use user-provided max/min values, or default to 0-10000, consistent with image_view's default.
    // References:
    // https://github.com/ros-perception/image_pipeline/blob/42266892502427eb566a4dffa61b009346491ce7/image_view/src/nodes/image_view.cpp#L80-L88
    // https://github.com/ros-visualization/rqt_image_view/blob/fe076acd265a05c11c04f9d04392fda951878f54/src/rqt_image_view/image_view.cpp#L582
    // https://github.com/ros-visualization/rviz/blob/68b464fb6571b8760f91e8eca6fb933ba31190bf/src/rviz/image/ros_image_texture.cpp#L114
    const minValue = options?.minValue ?? 0;
    let maxValue = options?.maxValue ?? 10000;
    if (maxValue === minValue) {
        maxValue = minValue + 1;
    }

    let outIdx = 0;
    for (let i = 0; i < width * height * 2; i += 2) {
        let val = view.getUint16(i, !isBigendian);

        val = ((val - minValue) / (maxValue - minValue)) * 255;

        output[outIdx++] = val;
        output[outIdx++] = val;
        output[outIdx++] = val;
        output[outIdx++] = 255;
    }
}


  export function makeSpecializedDecodeBayer(tl, tr, bl, br) {
    return function(data, width, height, output) {
      for (let i = 0; i < height / 2; i++) {
        let inIdx = i * 2 * width;
        let outTopIdx = i * 2 * width * 4;
        let outBottomIdx = (i * 2 + 1) * width * 4;
        for (let j = 0; j < width / 2; j++) {
          const tlValue = data[inIdx++];
          const trValue = data[inIdx++];
          const blValue = data[inIdx + width - 2];
          const brValue = data[inIdx + width - 1];
  
          const r = tl === 'r' ? tlValue : brValue;
          const g0 = tl === 'g0' ? tlValue : tr === 'g0' ? trValue : bl === 'g0' ? blValue : brValue;
          const g1 = tl === 'g1' ? tlValue : tr === 'g1' ? trValue : bl === 'g1' ? blValue : brValue;
          const b = tl === 'b' ? tlValue : brValue;
  
          // Top row
          output[outTopIdx++] = r;
          output[outTopIdx++] = g0;
          output[outTopIdx++] = b;
          output[outTopIdx++] = 255;
  
          output[outTopIdx++] = r;
          output[outTopIdx++] = g0;
          output[outTopIdx++] = b;
          output[outTopIdx++] = 255;
  
          // Bottom row
          output[outBottomIdx++] = r;
          output[outBottomIdx++] = g1;
          output[outBottomIdx++] = b;
          output[outBottomIdx++] = 255;
  
          output[outBottomIdx++] = r;
          output[outBottomIdx++] = g1;
          output[outBottomIdx++] = b;
          output[outBottomIdx++] = 255;
        }
      }
    };
  }
  
  export const decodeBayerRGGB8 = makeSpecializedDecodeBayer("r", "g0", "g1", "b");
  export const decodeBayerBGGR8 = makeSpecializedDecodeBayer("b", "g0", "g1", "r");
  export const decodeBayerGBRG8 = makeSpecializedDecodeBayer("g0", "b", "r", "g1");
  export const decodeBayerGRBG8 = makeSpecializedDecodeBayer("g0", "r", "b", "g1");