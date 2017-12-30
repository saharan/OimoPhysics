OimoPhysics
---

A lightweight 3D physics engine.

## [API Documentation](https://saharan.github.io/OimoPhysics/)

## Demos
<a href="http://el-ement.com/etc/oimo/demos/"><img src="http://el-ement.com/etc/oimo/demos/thumbs.png"></a>
* Press `E` or `Q` to change demos
* Click or tap text to control

## Features
* Written in Haxe
* Exported as JavaScript (see [bin/js/](./bin/js))
	* Public classes and methods will be exposed through `windwow.OIMO`.
	* e.g. `new OIMO.Vec3(1, 2, 3)` to create an instance of `Vec3` class.
* Rigid body motion types
	* Dynamic
	* Static
	* Kinematic
* Fast collision detection with bounding volume hierarchy (BVH)
* Contacts with friction and restitution
* Collision geometries
	* Sphere
	* Box
	* Cylinder
	* Cone
	* Capsule
	* Convex hull
* Joints with springs, limits and motors
	* Spherical (a.k.a. ball and socket, point to point)
	* Revolute (a.k.a. hinge)
	* Cylindrical
	* Prismatic (a.k.a. slider)
	* Universal
	* Ragdoll (a.k.a. cone twist, character)
* Breakable joints
* Constraint solvers
	* Direct block MLCP solver
	* Projected Gauss-Seidel solver
* Sleepings with island splittings
* Collision event callbacks
* Collision filterings
* Collision queries
	* AABB query
	* ray casting
	* convex casting

## Compile JavaScript demos in Haxe
* Use Haxe 4.0.0 or later
* main class: `demo.js.DemoJS`
* Try enabling compiler options if fails
	* `-D analyzer`
	* `-D eval-stack`

## License
The MIT License

---

### Old Version
* Written in ActionScript 3.0
* Supports spheres and boxes as collision shapes
* Supports various joints (ball and socket, distance, hinge, prismatic, etc...)
* Fast and stable collision solver
* Available in [old/](./old) directory
