import * as THREE from 'three'

import BOX2D from '@cocos/box2d';

export class PhysicsRayCastCallback extends BOX2D.RayCastCallback {
	ReportFixture (fixture: BOX2D.Fixture, point: BOX2D.Vec2, normal: BOX2D.Vec2, fraction: number): any {
		//console.log("ReportFixture", fixture, point, normal, fraction);
		return 0;
	}
}

export default class Box2dSceneRaycast extends THREE.Scene
{
	//private readonly camera: THREE.PerspectiveCamera

	//private container;
	// private stats : Stats | null = null;
	private inited = false;
	private camera;
	//private mesh;
	// private b2MapingThree = {};
	//private ramp_0, ramp_1, floor, ramp_geometry, material_red, material_green, ball_geometry, ball_material, large_ball_geometry, large_ball_material;
	private ramp_0 :any;
	private ramp_1 :any;
	private floor :any;
	private ramp_geometry :any;
	private material_red :any;
	private material_green :any;
	private ball_geometry :any;
	private ball_material :any;
	private large_ball_geometry :any;
	private large_ball_material :any;
	private RBCount = 0;
	private RBActiveCount = 0;
	private RBArray: BOX2D.Body[] = [];

	private box2dInited = false;
	private world: any;
	private bodyDef: any; // `bodyDef` will describe the type of bodies we're creating
	private fixDef: any;
	private callback: any;
	private tempVec2_1: any;
	private tempVec2_2: any;

	constructor(camera: THREE.PerspectiveCamera)
	{
		super()
		this.camera = camera
	}

	async initialize() {
		if(this.inited) return;
		this.inited = true;
		// performance monitor

		//this = new THREE.Scene();
		this.background = new THREE.Color( 0x050505 );
		// scene.fog = new THREE.Fog( 0x050505, 2000, 4500 );
		//scene adds lights
		this.add( new THREE.AmbientLight( 0x444444 ) );
		const light1 = new THREE.DirectionalLight( 0xffffff, 0.5 );
		light1.position.set( 1, 1, 1 );
		this.add( light1 );
		const light2 = new THREE.DirectionalLight( 0xffffff, 1.5 );
		light2.position.set( 0, - 1, 0 );
		this.add( light2 );

		//camera = new THREE.PerspectiveCamera( 5, window.innerWidth / window.innerHeight, 10, 4000 );
		this.camera.position.set( -10, 30, 3500 );
		this.camera.lookAt( this.position ); // Look at the center of the scene

		// renderer = new THREE.WebGLRenderer();
		// renderer.setPixelRatio( window.devicePixelRatio );
		// renderer.setSize( window.innerWidth, window.innerHeight );
		// renderer.outputEncoding = THREE.sRGBEncoding;

		// this.container = document.getElementById( 'container' );
		// this.container.appendChild( renderer.domElement );
		// window.addEventListener( 'resize', onWindowResize );

		//add objects to scene
		this.ramp_geometry= new THREE.BoxGeometry( 200, 1, 10 ),
		this.material_red = new THREE.MeshLambertMaterial({ color: 0xdd0000}),
		this.material_green = new THREE.MeshLambertMaterial({ color: 0x00bb00});
		

		this.ball_geometry = new THREE.SphereGeometry( 3 ); // Create the ball geometry with a radius of `3`
		this.ball_material = new THREE.MeshLambertMaterial({ color: 0x0000ff}); // Balls will be blue
		this.large_ball_geometry = new THREE.SphereGeometry( 4 ); // Create the ball geometry with a radius of `4`
		this.large_ball_material = new THREE.MeshLambertMaterial({ color: 0x00ff00}); // Large balls are be green
	}

	private initBox2d() {
		if(this.box2dInited) return;
		this.box2dInited = true;

		//box2d test
		//Create the physics world
		this.world = new BOX2D.World(
			new BOX2D.Vec2(0, -20), // Gravity
		);

		this.tempVec2_1 = new BOX2D.Vec2(0, 0);
		this.tempVec2_2 = new BOX2D.Vec2(0, 0);
		this.callback = new PhysicsRayCastCallback();

		this.bodyDef = new BOX2D.BodyDef(); // `bodyDef` will describe the type of bodies we're creating
		// Objects defined in this function are all static
		this.bodyDef.type = 0;//BOX2D.BodyType.b2_staticBody; 

		// FixtureDef
		this.fixDef = new BOX2D.FixtureDef();
		this.fixDef.density = 1.0;
		this.fixDef.friction = 0.3;
		this.fixDef.restitution = 0.3;

		// // position the ramp 0
		// if(1)
		// {
		// 	this.ramp_0 = new THREE.Mesh( this.ramp_geometry, this.material_red );
		// 	this.add( this.ramp_0 );

		// 	let rampPos = new BOX2D.Vec2(-200, 0);
		// 	this.bodyDef.position = rampPos;
		// 	// bodyDef.position.SetX(-20); //https://github.com/emscripten-core/emscripten/issues/17573
		// 	// bodyDef.position.SetY(25); 
		// 	this.ramp_0.position.x = rampPos.x;
		// 	this.ramp_0.position.y = rampPos.y;
		// 	this.bodyDef.angle = this.ramp_0.rotation.z = -Math.PI / 2;
		// 	//fixDef.shape = new BOX2D.PolygonShape();
		// 	//fixDef.shape.SetAsBox( 25, 1 ); // "25" = half width of the ramp, "1" = half height
		// 	let shape0 = new BOX2D.PolygonShape();
		// 	this.fixDef.shape = (shape0);
		// 	shape0.SetAsBox( 100, 1 ); // "25" = half width of the ramp, "1" = half height
		// 	//bodyDef.userData = ramp_0; // Keep a reference to `ramp_1`
		// 	let ramp0RB = this.world.CreateBody( this.bodyDef );
		// 	//ramp0RB.userData = ramp_0;
		// 	ramp0RB.m_userData = this.ramp_0;
		// 	//b2MapingThree[ramp0RB.$$.ptr] = ramp_0;
		// 	ramp0RB.CreateFixture( this.fixDef ); // Add this physics body to the world
		// }

		// // // position the ramp 1
		// if(1)
		// {
		// 	this.ramp_1 = new THREE.Mesh( this.ramp_geometry, this.material_red );
		// 	this.add( this.ramp_1 );

		// 	let rampPos = new BOX2D.Vec2(200, 0);
		// 	this.bodyDef.position = rampPos;
		// 	// bodyDef.position.SetX(25);
		// 	// bodyDef.position.SetY(5);
		// 	this.ramp_1.position.x = rampPos.x;
		// 	this.ramp_1.position.y = rampPos.y;
		// 	this.bodyDef.angle = this.ramp_1.rotation.z = Math.PI / 2;
		// 	// fixDef.shape = new b2PolygonShape;
		// 	// fixDef.shape.SetAsBox( 25, 1 ); // "25" = half width of the ramp, "1" = half height
		// 	let shape1 = new BOX2D.PolygonShape();
		// 	this.fixDef.shape = (shape1);
		// 	shape1.SetAsBox( 100, 1 ); // "25" = half width of the ramp, "1" = half height
		// 	//bodyDef.userData = ramp_1; // Keep a reference to `ramp_2`
		// 	let ramp1RB = this.world.CreateBody( this.bodyDef );
		// 	ramp1RB.m_userData = this.ramp_1;
		// 	//b2MapingThree[ramp1RB.$$.ptr] = ramp_1;
		// 	ramp1RB.CreateFixture( this.fixDef ); // Add this physics body to the world
		// 	this.RBArray.push(ramp1RB);

		// }

		// // Create the floor
		// {
		// 	this.floor = new THREE.Mesh( new THREE.BoxGeometry( 400, 1, 10 ), this.material_red );
		// 	//floor = new THREE.Mesh( new THREE.PlaneGeometry( 100, 50 ), material_red );
		// 	this.add( this.floor );

		// 	let floorPos = new BOX2D.Vec2(0, -100);
		// 	this.bodyDef.position = floorPos;
		// 	// bodyDef.position.SetX(0);
		// 	// bodyDef.position.SetY(-45);
		// 	this.floor.position.x = floorPos.x;
		// 	this.floor.position.y = floorPos.y; // position the floor
		// 	this.bodyDef.angle = 0;
		// 	// fixDef.shape = new b2PolygonShape;
		// 	// fixDef.shape.SetAsBox( 50, .1 ); // "50" = half width of the floor, ".1" = half height
		// 	let shape2 = new BOX2D.PolygonShape();
		// 	this.fixDef.shape = shape2;
		// 	shape2.SetAsBox( 200, 1 ); // "25" = half width of the ramp, "1" = half height
		// 	//bodyDef.userData = floor; // Keep a reference to `floor`
		// 	//world.CreateBody( bodyDef ).CreateFixture( fixDef ); // Add this physics body to the world
		// 	//let floorRB = world.CreateBody( bodyDef );
		// 	let floorRB = this.world.CreateBody( this.bodyDef );
		// 	//let floorRB = this.world.GetBodyList();
		// 	floorRB.m_userData = this.floor;
		// 	// this.b2MapingThree[floorRB] = this.floor;
		// 	floorRB.CreateFixture( this.fixDef );
		// 	//add floorRB to RBArray
		// 	this.RBArray.push(floorRB);
		// }
		for(let i = 0; i < 5000; i++){
			this.addBall();
		}
	}

	// async initialize()
	// {
	// 	// load a shared MTL (Material Template Library) for the targets
	// 	const targetMtl = await this.mtlLoader.loadAsync('assets/targetA.mtl')
	// 	targetMtl.preload()

	// 	this.bulletMtl = await this.mtlLoader.loadAsync('assets/foamBulletB.mtl')
	// 	this.bulletMtl.preload()

	// 	// create the 4 targets
	// 	const t1 = await this.createTarget(targetMtl)
	// 	t1.position.x = -1
	// 	t1.position.z = -3

	// 	const t2 = await this.createTarget(targetMtl)
	// 	t2.position.x = 1
	// 	t2.position.z = -3

	// 	const t3 = await this.createTarget(targetMtl)
	// 	t3.position.x = 2
	// 	t3.position.z = -3

	// 	const t4 = await this.createTarget(targetMtl)
	// 	t4.position.x = -2
	// 	t4.position.z = -3

	// 	this.add(t1, t2, t3, t4)
	// 	this.targets.push(t1, t2, t3, t4)

	// 	this.blaster = await this.createBlaster()
	// 	this.add(this.blaster)

	// 	this.blaster.position.z = 3
	// 	this.blaster.add(this.camera)

	// 	this.camera.position.z = 1
	// 	this.camera.position.y = 0.5

	// 	const light = new THREE.DirectionalLight(0xFFFFFF, 1)
	// 	light.position.set(0, 4, 2)

	// 	this.add(light)

	// 	document.addEventListener('keydown', this.handleKeyDown)
	// 	document.addEventListener('keyup', this.handleKeyUp)
	// }
	
	private updateBox2d() {
		if(!this.box2dInited) return;

		//addBall();

		// var delta, now = (new Date()).getTime();
		var delta = 1 / 60;
		// if ( time_last_run ) {
		// 	delta = ( now - time_last_run ) / 1000;
		// } else {
		// 	delta = 1 / 60;
		// }
		// time_last_run = now; 
		
		this.world.Step(
			delta * 2, // double the speed of the simulation
			10,        // velocity iterations
			10         // position iterations
		);

		// Update the scene objects
		let bodyList = this.world.GetBodyList();
		this.RBCount = 0;
		this.RBActiveCount = 0;
		//iterate all elements of this.RBArray
		for(let i = 0; i < this.RBArray.length; i++){
			this.RBCount++;
			let b2RB = this.RBArray[i];
			let mesh = b2RB.m_userData;
			if ( b2RB && mesh ) {
				// Nice and simple, we only need to work with 2 dimensions
				let position = b2RB.GetPosition();
				mesh.position.x = position.x;
				mesh.position.y = position.y;
				// GetAngle() function returns the rotation in radians
				mesh.rotation.z = b2RB.GetAngle();
				//free object not visible
				if(mesh.position.y < -110) {
					this.remove(mesh);
					// mesh = null;
					this.world.DestroyBody(b2RB);
					// delete this.b2MapingThree[b2RB.$$.ptr];
				}
				if(b2RB.IsAwake()){
					this.RBActiveCount++;
				}
			}
		}

		// while(bodyList) {
		// 	this.RBCount++;
		// 	let b2RB = bodyList;
		// 	//let mesh = this.b2MapingThree[b2RB];
		// 	let mesh = b2RB.userData;
		// 	if ( b2RB && mesh ) {
		// 		// Nice and simple, we only need to work with 2 dimensions
		// 		let position = b2RB.GetPosition();
		// 		mesh.position.x = position.x;
		// 		mesh.position.y = position.y;
				
		// 		// GetAngle() function returns the rotation in radians
		// 		mesh.rotation.z = b2RB.GetAngle();
		// 		bodyList = b2RB.GetNext();

		// 		//free object not visible
		// 		if(mesh.position.y < -100) {
		// 			this.remove(mesh);
		// 			// mesh = null;
		// 			this.world.DestroyBody(b2RB);
		// 			// delete this.b2MapingThree[b2RB.$$.ptr];
		// 		}
		// 		if(b2RB.IsAwake()){
		// 			this.RBActiveCount++;
		// 		}
		// 	}
		// 	else{
		// 		console.log("b2RB not valid", b2RB);
		// 	}
		// }
		// while(bodyList) {
		// 	this.RBCount++;
		// 	let b2RB = bodyList;
		// 	//let mesh = this.b2MapingThree[b2RB];
		// 	let mesh = b2RB.userData;
		// 	if ( b2RB && mesh ) {
		// 		// Nice and simple, we only need to work with 2 dimensions
		// 		let position = b2RB.GetPosition();
		// 		mesh.position.x = position.x;
		// 		mesh.position.y = position.y;
				
		// 		// GetAngle() function returns the rotation in radians
		// 		mesh.rotation.z = b2RB.GetAngle();
		// 		bodyList = b2RB.GetNext();

		// 		//free object not visible
		// 		if(mesh.position.y < -100) {
		// 			this.remove(mesh);
		// 			// mesh = null;
		// 			this.world.DestroyBody(b2RB);
		// 			// delete this.b2MapingThree[b2RB.$$.ptr];
		// 		}
		// 		if(b2RB.IsAwake()){
		// 			this.RBActiveCount++;
		// 		}
		// 	}
		// 	else{
		// 		console.log("b2RB not valid", b2RB);
		// 	}
		// }
		
		console.log("rigid body count =",this.RBCount, "active = ", this.RBActiveCount);
		//var object = world.GetBody(i), mesh, position;
		// while ( object ) {
		// 	mesh = object.GetUserData();
			
		// 	if ( mesh ) {
		// 		// Nice and simple, we only need to work with 2 dimensions
		// 		position = object.GetPosition();
		// 		mesh.position.x = position.x;
		// 		mesh.position.y = position.y;
				
		// 		// GetAngle() function returns the rotation in radians
		// 		mesh.rotation.z = object.GetAngle();
		// 	}
			
		// 	object = object.GetNext(); // Get the next object in the scene
		// }
	}

	private addBall() {
		if(!this.box2dInited) return;
		// if(this.RBCount > 2000) return;
		var ball;
						
		let shape = new BOX2D.CircleShape();
		if ( Math.random() >= .05 ) {
			ball = new THREE.Mesh( this.ball_geometry, this.ball_material );
			shape.m_radius =  3;
		} else {
			ball = new THREE.Mesh( this.large_ball_geometry, this.large_ball_material );
			shape.m_radius = 4;
		}
		this.fixDef.shape= shape;
		//shape = null;

		this.add( ball );
		this.bodyDef.type = 0;//BOX2D.BodyType.b2_staticBody; // balls can move

		let randomX = (Math.random() * 2 - 1) * 400;
		let randomY = (Math.random() * 2 - 1) * 100;
		let ballPos = new BOX2D.Vec2(randomX, randomY);
		//let ballPos = new BOX2D.Vec2(Math.random() * 40 - 20, 200);
		this.bodyDef.position = ballPos;
		// bodyDef.position.SetY(50);
		// bodyDef.position.SetX(Math.random() * 40 - 20); // Random positon between -20 and 20

		ball.position.x = ballPos.x;
		ball.position.y = ballPos.y;
		//bodyDef.userData = ball; // Keep a reference to `ball`
		//world.CreateBody( bodyDef ).CreateFixture( fixDef ); // Add this physics body to the world
		//let ballRB = world.CreateBody( bodyDef );//this ballRB is a new object, not the same sa inside the world
		let ballRB = this.world.CreateBody( this.bodyDef );
		//let ballRB = this.world.GetBodyList();
		// console.log('ballRB1', ballRB1);
		// console.log(ballRB1.GetWorld());
		// console.log('ballRB',ballRB);
		// console.log(ballRB.GetWorld());

		ballRB.m_userData = ball;
		//this.b2MapingThree[ballRB] = ball;
		ballRB.CreateFixture( this.fixDef ); // Add this physics body to the world
		//ballPos = null;
		this.RBArray.push(ballRB);
	}

	private raycastTest() {
		if(!this.box2dInited) return;
		this.tempVec2_1.x = (-2000);
		this.tempVec2_1.y = (10);
		this.tempVec2_2.x = (1000);
		this.tempVec2_2.y = (10);

		let time0 = Date.now();
		for(let i = 0; i < 2000; i++){
			this.tempVec2_1.x = (-2000 + i);
			//this.world.RayCast(this.callback, this.tempVec2_1, this.tempVec2_2);
			this.world.RayCastOne(this.tempVec2_1, this.tempVec2_2);
		}
		let time1 = Date.now();
		console.log("raycastTest time = ", time1 - time0);
	}

	update()
	{
		// for(let i = 0; i < 2; i++){
		// 	this.addBall();
		// }
		this.initBox2d();
		this.raycastTest();
		//this.updateBox2d();
	}
}