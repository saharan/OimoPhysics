OimoPhysics 1.2.1
---

A lightweight 3D physics engine.

## [API Documentation](https://saharan.github.io/OimoPhysics/)

## Demos
<a href="http://el-ement.com/etc/oimo/demos/"><img src="http://el-ement.com/etc/oimo/demos/thumbnail.png"></a>
* Press `E` or `Q` to change demos
* Click or tap text to control

## Features
* Written in Haxe
* Exported as JavaScript (see [bin/js/](./bin/js))
	* Public classes and methods will be exposed through `window.OIMO`.
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
	* Generic (a.k.a. 6-DoF joint)
* Breakable joints
* Constraint solvers
	* Direct block MLCP solver
	* Projected Gauss-Seidel solver
* Sleepings with island splittings
* Rotation limits
* Collision event callbacks
* Collision filterings
* Collision queries
	* AABB query
	* ray casting
	* convex casting

## Compilations
Haxe 4.0.0 or later is required. (recommended: Haxe 4.0.5 or later)
* Use `build-js.hxml` to compile for JavaScript library.
* Use `build-js-demos.hxml` to compile JavaScript demos.
* Use `build-doc.hxml` to generate API documentation. [dox](https://github.com/HaxeFoundation/dox) is required.

## License
The MIT License

---

### Old Version
* Written in ActionScript 3.0
* Supports spheres and boxes as collision shapes
* Supports various joints (ball and socket, distance, hinge, prismatic, etc...)
* Fast and stable collision solver
* Available in [old/](./old) directory
