/// <reference path="./b2.d.ts" />

// declare module 'external:emscripten/box2d/box2d.release.asm.js' {
//   export default PhysX;
// }

declare module 'external:emscripten/box2d/box2d.release.wasm.js' {
    export default BOX2D;
}

// tslint:disable
declare function BOX2D (moduleOptions?: any): Promise<void>;
