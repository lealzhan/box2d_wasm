import * as THREE from 'three'
// import BlasterScene from './BlasterScene'
import Box2DScene from './Box2dScene'
// import Box2DSceneRaycast from './Box2dSceneRaycast'
import Stats from 'three/examples/jsm/libs/stats.module'


const width = window.innerWidth
const height = window.innerHeight

const renderer = new THREE.WebGLRenderer({
	canvas: document.getElementById('app') as HTMLCanvasElement
})
renderer.setSize(width, height)

const mainCamera = new THREE.PerspectiveCamera(10, width / height, 10, 5000)

let stats = Stats();
document.body.appendChild(stats.dom);

const scene = new Box2DScene(mainCamera)
// const scene = new Box2DSceneRaycast(mainCamera)
scene.initialize()

function tick()
{
	scene.update()
	renderer.render(scene, mainCamera)
	stats.update();
	requestAnimationFrame(tick)
}

tick()
