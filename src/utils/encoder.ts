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


export function encodeMono8(input: Uint8Array, width: number, height: number, output: Uint8Array) {
    let inIdx = 0;
    let outIdx = 0;
  
    for (let i = 0; i < width * height; i++) {
      const r = input[inIdx++];
      const g = input[inIdx++];
      const b = input[inIdx++];
  
      // Convert to grayscale using the formula: Y = 0.299R + 0.587G + 0.114B
      const gray = Math.round(0.299 * r + 0.587 * g + 0.114 * b);
  
      output[outIdx++] = gray;
    }
  }

