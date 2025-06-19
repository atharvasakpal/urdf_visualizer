import { MediaPipeHandController } from './mediapipe_humanoid_controller.js';

class ProfessionalURDFViewer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.robot = null; // The root THREE.Group for the entire robot
        this.joints = {}; // Stores parsed joint data
        this.links = {};  // Stores THREE.Group for each link
        this.jointControls = {};
        this.axesHelpers = [];
        this.meshFiles = new Map(); // Stores File objects, keyed by base filename
        this.loadedGeometries = new Map(); // Stores loaded THREE.BufferGeometry, keyed by original mesh path
        this.gridHelper = null;
        this.autoRotateEnabled = false;
        this.stats = { links: 0, joints: 0, triangles: 0 };
        this.handController = null; // Ensure this property is declared in the class or constructor

        // this.controls = null;

        this.init();
        this.setupEventListeners();
        this.animate();
    }
    
    init() {
        // Scene setup with professional lighting
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0a14);
        this.scene.fog = new THREE.Fog(0x0a0a14, 10, 50);

        // Camera setup
        this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
        this.camera.position.set(4, 4, 4);
        this.camera.lookAt(0, 0, 0);

        // Renderer setup with enhanced quality
        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: true,
            powerPreference: "high-performance"
        });
        this.renderer.setSize(window.innerWidth - 350, window.innerHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.2;

        const viewportDiv = document.getElementById('viewport');
        if (viewportDiv) {
            viewportDiv.appendChild(this.renderer.domElement);
        }
        // Ensure you have removed the duplicate line here if it was present:
        // document.getElementById('viewport').appendChild(this.renderer.domElement);

        // Professional lighting setup
        this.setupLighting();

        // Grid
        this.gridHelper = new THREE.GridHelper(10, 20, 0x444466, 0x222233);
        this.gridHelper.material.transparent = true;
        this.gridHelper.material.opacity = 0.3;
        this.scene.add(this.gridHelper);

        // Mouse controls
        this.setupMouseControls();

        // Keyboard controls
        this.setupKeyboardControls();

        // MediaPipe Hand Controller Initialization
        const webcamVideoElement = document.getElementById('webcam');
        if (webcamVideoElement) {
            this.handController = new MediaPipeHandController(this, webcamVideoElement);
        } else {
            console.error("Webcam video element not found in the DOM.");
        }
    }


    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404060, 0.3);
        this.scene.add(ambientLight);
        
        // Main directional light
        const mainLight = new THREE.DirectionalLight(0xffffff, 1.2);
        mainLight.position.set(10, 15, 5);
        mainLight.castShadow = true;
        mainLight.shadow.mapSize.width = 2048;
        mainLight.shadow.mapSize.height = 2048;
        mainLight.shadow.camera.near = 0.5;
        mainLight.shadow.camera.far = 50;
        mainLight.shadow.camera.left = -10;
        mainLight.shadow.camera.right = 10;
        mainLight.shadow.camera.top = 10;
        mainLight.shadow.camera.bottom = -10;
        mainLight.shadow.bias = -0.0005;
        this.scene.add(mainLight);
        
        // Fill light
        const fillLight = new THREE.DirectionalLight(0x6366f1, 0.4);
        fillLight.position.set(-5, 5, -5);
        this.scene.add(fillLight);
        
        // Rim light
        const rimLight = new THREE.DirectionalLight(0xa855f7, 0.3);
        rimLight.position.set(0, -5, 10);
        this.scene.add(rimLight);
        
        // Point lights for dramatic effect
        const pointLight1 = new THREE.PointLight(0x6366f1, 0.5, 20);
        pointLight1.position.set(5, 3, 5);
        this.scene.add(pointLight1);
        
        const pointLight2 = new THREE.PointLight(0xa855f7, 0.3, 15);
        pointLight2.position.set(-3, 2, -3);
        this.scene.add(pointLight2);
    }
    
    setupMouseControls() {
        let isDragging = false;
        let previousMousePosition = { x: 0, y: 0 };
        // const element = this.renderer.domElement;
        const element = document.getElementById('viewport');
        
        element.addEventListener('mousedown', (e) => {
            isDragging = true;
            previousMousePosition = { x: e.clientX, y: e.clientY };
            element.style.cursor = 'grabbing';
        });
        
        element.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const deltaMove = {
                    x: e.clientX - previousMousePosition.x,
                    y: e.clientY - previousMousePosition.y
                };
                
                const spherical = new THREE.Spherical();
                spherical.setFromVector3(this.camera.position);
                
                spherical.theta -= deltaMove.x * 0.01;
                spherical.phi += deltaMove.y * 0.01;
                spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));
                
                this.camera.position.setFromSpherical(spherical);
                this.camera.lookAt(0, 0, 0);
                
                previousMousePosition = { x: e.clientX, y: e.clientY };
            }
        });
        
        element.addEventListener('mouseup', () => {
            isDragging = false;
            element.style.cursor = 'grab';
        });
        
        element.addEventListener('mouseleave', () => {
            isDragging = false;
            element.style.cursor = 'default';
        });
        
        element.addEventListener('wheel', (e) => {
            e.preventDefault();
            const scale = e.deltaY > 0 ? 1.1 : 0.9;
            this.camera.position.multiplyScalar(scale);
            const distance = this.camera.position.length();
            if (distance > 30) this.camera.position.normalize().multiplyScalar(30);
            if (distance < 1) this.camera.position.normalize().multiplyScalar(1);
        });
        
        element.style.cursor = 'grab';
    }
    
    resetCamera() {
        this.camera.position.set(4, 4, 4);
        this.camera.lookAt(0, 0, 0);
    }

    setupKeyboardControls() {
        document.addEventListener('keydown', (e) => {
            switch(e.key.toLowerCase()) {
                case 'r':
                    this.resetCamera();
                    break;
                case 'g':
                    document.getElementById('showGrid').click();
                    break;
                case 'w':
                    document.getElementById('wireframe').click();
                    break;
            }
        });
    }
    
    setupEventListeners() {
        const urdfInput = document.getElementById('urdfInput');
        const meshInput = document.getElementById('meshInput');
        const loadButton = document.getElementById('loadButton');
        
        const checkInputs = () => {
            loadButton.disabled = !urdfInput.files.length;
            
            // Update file input UI
            const urdfWrapper = document.getElementById('urdfWrapper');
            if (urdfInput.files.length) {
                urdfWrapper.classList.add('has-files');
                urdfWrapper.querySelector('.file-input-text').innerHTML = 
                    `<strong>Selected:</strong> ${urdfInput.files[0].name}`;
            } else {
                urdfWrapper.classList.remove('has-files');
                urdfWrapper.querySelector('.file-input-text').innerHTML = 
                    '<strong>Click to select</strong> or drag URDF file here';
            }
        };
        
        urdfInput.addEventListener('change', checkInputs);
        
        meshInput.addEventListener('change', (e) => {
            this.processMeshFiles(e.target.files);
            
            // Update mesh input UI
            const meshWrapper = document.getElementById('meshWrapper');
            if (e.target.files.length) {
                meshWrapper.classList.add('has-files');
                meshWrapper.querySelector('.file-input-text').innerHTML = 
                    `<strong>Selected:</strong> ${e.target.files.length} mesh files`;
            } else {
                meshWrapper.classList.remove('has-files');
                meshWrapper.querySelector('.file-input-text').innerHTML = 
                    '<strong>Click to select</strong> mesh files or drag here';
            }
        });
        
        loadButton.addEventListener('click', () => {
            this.loadRobot();
        });
        
        document.getElementById('showAxes').addEventListener('change', (e) => {
            this.toggleAxes(e.target.checked);
        });
        
        document.getElementById('wireframe').addEventListener('change', (e) => {
            this.toggleWireframe(e.target.checked);
        });
        
        document.getElementById('showGrid').addEventListener('change', (e) => {
            this.toggleGrid(e.target.checked);
        });
        
        document.getElementById('autoRotate').addEventListener('change', (e) => {
            this.autoRotateEnabled = e.target.checked;
        });
    }
    
    processMeshFiles(files) {
        this.meshFiles.clear(); // Clear previous mesh files
        for (let file of files) {
            // Store files by their base filename for easier lookup from URDF paths
            const baseName = file.name.split('/').pop().split('.')[0]; // Handles paths and extensions
            this.meshFiles.set(file.name.split('/').pop(), file); // Store by full filename (e.g., "mesh.stl")
            this.meshFiles.set(baseName, file); // Store by base name (e.g., "mesh")
        }
        this.updateStatus(`Loaded ${files.length} potential mesh files for lookup.`, 'success');
    }





    //new code 








    
    // In urdf_script.js, inside ProfessionalURDFViewer class

    async loadRobot() {
        const urdfFile = document.getElementById('urdfInput').files[0];
        if (!urdfFile) {
            this.updateStatus('Please select a URDF file.', 'error');
            return;
        }
    
        this.updateStatus('Loading robot...', 'loading');
        document.getElementById('loadButtonText').innerHTML =
            '<div class="loading-spinner"></div>Loading...';
    
        // Clear previous robot if any
        if (this.robot) {
            this.scene.remove(this.robot);
            this.robot.traverse(obj => {
                if (obj.geometry) obj.geometry.dispose();
                if (obj.material) obj.material.dispose();
            });
        }
        this.robot = new THREE.Group();
        this.robot.name = "Robot_Root";
        this.scene.add(this.robot);
    
        this.joints = {};
        this.links = {};
        this.jointControls = {};
        this.axesHelpers.forEach(helper => helper.parent.remove(helper)); // Remove old axes
        this.axesHelpers = [];
        this.loadedGeometries.clear(); // Clear cached geometries
    
        try {
            const urdfContent = await this.readFile(urdfFile);
            await this.parseURDF(urdfContent);
    
            // --- OPTION 1: Set Initial Joint Pose (More granular, requires setInitialRobotPose() method) ---
            // Uncomment the line below if you want to set specific joint angles to articulate the robot.
            // You'll need to define the setInitialRobotPose() method in your class.
            // this.setInitialRobotPose();
            // -------------------------------------------------------------------------------------------------
    
            // --- OPTION 2: Apply Universal Rotation to the Entire Robot (Less granular, simpler) ---
            // This rotates the whole robot group. Useful for correcting overall orientation (e.g., standing it up).
            // Experiment with the rotation axis (x, y, or z) and the angle (in radians).
            // Common examples:
            // this.robot.rotation.x = -Math.PI / 2; // Rotate -90 degrees around X-axis (if robot is flat on its back)
            // this.robot.rotation.y = Math.PI;    // Rotate 180 degrees around Y-axis (if robot is facing wrong way)
            // this.robot.rotation.z = Math.PI / 2; // Rotate 90 degrees around Z-axis
            this.robot.rotation.x = -Math.PI / 2; // Example: Try this for -90 degrees around X. Adjust as needed.
            // You might also need to adjust its global position if the rotation causes it to go below the ground plane:
            // this.robot.position.y = 0.5; // Example: move it up by 0.5 units if needed
            // -------------------------------------------------------------------------------------------------
    
            this.updateStatus('Robot loaded successfully!', 'success');
            document.getElementById('loadButtonText').textContent = 'Load Robot';
    
            // Show joint controls if there are any
            if (Object.keys(this.joints).length > 0) {
                document.getElementById('jointSection').style.display = 'block';
            } else {
                document.getElementById('jointSection').style.display = 'none';
            }
    
            this.fitCameraToObject();
    
        } catch (error) {
            console.error('Error loading robot:', error);
            this.updateStatus('Error loading robot: ' + error.message, 'error');
            document.getElementById('loadButtonText').textContent = 'Load Robot';
        }
    }


    setInitialRobotPose() {
        // This function sets the initial positions for your robot's joints.
        // Replace 'joint_name_1', 'joint_name_2', etc., with the actual names from your URDF.
        // Adjust the rotation values (in radians) to achieve the desired "active" pose.
        // Remember: THREE.js uses radians for rotations (Math.PI = 180 degrees).

        // Example for a hypothetical robot with a few joints:
        if (this.joints['joint_shoulder_pan']) {
            // Rotate around the Z-axis by 45 degrees
            this.joints['joint_shoulder_pan'].rotation.z = Math.PI / 4;
        }

        if (this.joints['joint_shoulder_lift']) {
            // Rotate around the X-axis by -30 degrees
            this.joints['joint_shoulder_lift'].rotation.x = -Math.PI / 6;
        }

        if (this.joints['joint_elbow']) {
            // Rotate around the X-axis by 60 degrees
            this.joints['joint_elbow'].rotation.x = Math.PI / 3;
        }

        if (this.joints['joint_wrist_1']) {
            // Rotate around the Z-axis by 15 degrees
            this.joints['joint_wrist_1'].rotation.z = Math.PI / 12;
        }

        // Add more 'if' blocks for all relevant joints that need an initial pose.
        // You might need to experiment with the rotation axes (x, y, z) and values
        // to get the desired "active" look for your specific robot model.
    }



    
    readFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = e => resolve(e.target.result);
            reader.onerror = reject;
            reader.readAsText(file);
        });
    }
    
    async parseURDF(urdfContent) {
        const parser = new DOMParser();
        const xmlDoc = parser.parseFromString(urdfContent, "text/xml");
        
        const rootElement = xmlDoc.getElementsByTagName('robot')[0];
        if (!rootElement) {
            throw new Error("Invalid URDF: No 'robot' tag found.");
        }
    
        // Parse links first
        const links = xmlDoc.getElementsByTagName('link');
        for (let link of links) {
            await this.parseLink(link);
        }
        
        // Parse joints second
        const joints = xmlDoc.getElementsByTagName('joint');
        for (let joint of joints) {
            this.parseJoint(joint);
        }
    
        // Build the Three.js scene graph hierarchy
        // Find the root link (one that is not a child of any joint)
        let rootLinkName = null;
        const childLinkNames = new Set(Object.values(this.joints).map(j => j.child));
        for (const linkName in this.links) {
            if (!childLinkNames.has(linkName)) {
                rootLinkName = linkName;
                break;
            }
        }
    
        if (rootLinkName && this.links[rootLinkName]) {
            this.robot.add(this.links[rootLinkName]);
        } else if (Object.keys(this.links).length > 0) {
            // If no clear root, just add the first found link to the robot group,
            // this might not be perfectly hierarchical but prevents an empty scene.
            console.warn("Could not determine a clear root link. Adding first link to scene root.");
            this.robot.add(Object.values(this.links)[0]);
        }
    
        // Connect child links to parent links via joints
        for (const jointName in this.joints) {
            const joint = this.joints[jointName];
            const parentLinkGroup = this.links[joint.parent];
            const childLinkGroup = this.links[joint.child];
    
            if (parentLinkGroup && childLinkGroup) {
                parentLinkGroup.add(childLinkGroup);
    
                // Apply joint origin transform to the child link group
                // It's important that joint.origin is an object {xyz:[], rpy:[]} parsed in parseJoint
                // The applyOrigin function is designed to handle this object.
                if (joint.origin) { // Ensure joint.origin exists before passing
                    this.applyOrigin(childLinkGroup, joint.origin);
                }
    
                // Store initial relative position/rotation for prismatic/revolute joints
                joint.initialPosition = childLinkGroup.position.clone();
                joint.initialRotation = childLinkGroup.quaternion.clone();
    
                // Add axis helper to the child link's coordinate frame
                const axisHelper = new THREE.ArrowHelper(
                    new THREE.Vector3(...joint.axis).normalize(),
                    new THREE.Vector3(0, 0, 0), // At the child's origin relative to its parent
                    0.1, // length
                    0xff0000, // color
                    0.05, // headLength
                    0.02  // headWidth
                );
                childLinkGroup.add(axisHelper); // Add to the child's group
                this.axesHelpers.push(axisHelper);
            } else {
                console.warn(`Missing parent or child link for joint: ${jointName}`);
            }
        }
        
        this.createJointControls();
    }


    updateStatsDisplay() {
    document.getElementById('linkCount').textContent = this.stats.links;
    document.getElementById('jointCount').textContent = this.stats.joints;
    
    // Count triangles
    let triangleCount = 0;
    if (this.robot) {
        this.robot.traverse(child => {
            if (child.isMesh && child.geometry) {
                const positions = child.geometry.attributes.position;
                if (positions) {
                    triangleCount += positions.count / 3;
                }
            }
        });
    }
    this.stats.triangles = Math.floor(triangleCount);
    document.getElementById('triangleCount').textContent = this.stats.triangles;
  }

    async parseLink(linkElement) {
        const name = linkElement.getAttribute('name');
        const linkGroup = new THREE.Group();
        linkGroup.name = name;
        
        // Parse visual elements
        const visuals = linkElement.getElementsByTagName('visual');
        for (let visual of visuals) {
            const mesh = await this.createVisualMesh(visual);
            if (mesh) {
                linkGroup.add(mesh);
            }
        }
        
        this.links[name] = linkGroup;
        // Links are not added to the robot group yet, they will be attached via joints later.
    }

    async createVisualMesh(visual) {
        const geometryElement = visual.getElementsByTagName('geometry')[0];
        if (!geometryElement) return null;
        
        let geometry = null;
        let material = this.parseMaterial(visual.getElementsByTagName('material')[0]);
        
        // Check for mesh geometry
        const meshElement = geometryElement.getElementsByTagName('mesh')[0];
        if (meshElement) {
            geometry = await this.loadMeshGeometry(meshElement);
        } else {
            geometry = this.parseGeometry(geometryElement);
        }
        
        if (!geometry) return null;
        
        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;

        // Apply origin transform to the mesh within the link
        const origin = visual.getElementsByTagName('origin')[0];
        if (origin) {
            this.applyOrigin(mesh, origin);
        }
        
        return mesh;
    }

    async loadMeshGeometry(meshElement) {
        const filename = meshElement.getAttribute('filename');
        if (!filename) return null;
        
        // Handle package:// URI if present, otherwise assume relative path
        let meshPath = filename;
        if (meshPath.startsWith('package://')) {
            // Remove 'package://' and get the actual path relative to the package root
            meshPath = meshPath.substring('package://'.length);
            // For simplicity, we'll just take the last part of the path for lookup for now
            // A more robust system would involve mapping package names to actual base paths.
        }
        
        const meshBaseName = meshPath.split('/').pop();
        const meshExtension = meshBaseName.split('.').pop().toLowerCase();
        
        // Check if already loaded
        if (this.loadedGeometries.has(meshPath)) {
            // Return a clone to ensure each link gets its own geometry instance
            // This is important if you modify geometry (e.g., wireframe)
            return this.loadedGeometries.get(meshPath).clone();
        }
        
        // Try to find the mesh file in the files uploaded by the user
        let meshFile = this.meshFiles.get(meshBaseName);
        if (!meshFile) {
            console.warn(`Mesh file not found in provided files: ${meshPath}. Using fallback cube.`);
            return new THREE.BoxGeometry(0.1, 0.1, 0.1); // Fallback
        }
        
        try {
            const geometry = await this.loadMeshFile(meshFile, meshExtension);
            this.loadedGeometries.set(meshPath, geometry); // Cache the loaded geometry

            // Apply scale if specified by URDF
            const scale = meshElement.getAttribute('scale');
            if (scale && geometry) {
                const scaleValues = scale.split(' ').map(Number);
                const sx = scaleValues[0] !== undefined ? scaleValues[0] : 1;
                const sy = scaleValues[1] !== undefined ? scaleValues[1] : 1;
                const sz = scaleValues[2] !== undefined ? scaleValues[2] : 1;
                geometry.scale(sx, sy, sz);
            }
            
            return geometry;
        } catch (error) {
            console.error(`Error loading mesh ${meshPath}:`, error);
            this.updateStatus(`Error loading mesh ${meshBaseName}. Check console.`, 'error');
            return new THREE.BoxGeometry(0.1, 0.1, 0.1); // Fallback
        }
    }

    loadMeshFile(file, extension) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            
            reader.onload = (e) => {
                try {
                    let geometry;
                    if (extension === 'stl') {
                        const loader = new THREE.STLLoader();
                        geometry = loader.parse(e.target.result);
                    } else if (extension === 'obj') {
                        const loader = new THREE.OBJLoader();
                        // OBJLoader returns a Group, we need to extract the geometry
                        const objGroup = loader.parse(e.target.result);
                        if (objGroup.children.length > 0 && objGroup.children[0].isMesh) {
                            geometry = objGroup.children[0].geometry;
                        } else {
                            throw new Error("OBJ file did not contain a mesh with geometry.");
                        }
                    } else if (extension === 'dae') {
                        const loader = new THREE.ColladaLoader();
                        // ColladaLoader is async itself, might need a wrapper
                        // This part demonstrates an async loader within a promise:
                        loader.parse(e.target.result, (collada) => {
                            // Collada model usually contains a scene with various meshes
                            const meshes = [];
                            collada.scene.traverse((child) => {
                                if (child.isMesh) {
                                    meshes.push(child.geometry);
                                }
                            });
                            if (meshes.length > 0) {
                                // Combine geometries if multiple meshes are found, or take the first
                                // For simplicity, we'll just return the first geometry found.
                                // A more robust solution might merge them or create a Group.
                                resolve(meshes[0]);
                            } else {
                                reject(new Error(`DAE file "${file.name}" contained no meshes.`));
                            }
                        }, (xhr) => { /* progress */ }, (error) => {
                            reject(new Error(`Error loading DAE "${file.name}": ${error}`));
                        });
                        return; // DAE loader handles resolve/reject internally
                    } else {
                        throw new Error(`Unsupported mesh format: .${extension} for file ${file.name}`);
                    }
                    
                    // Ensure geometry has normals for lighting
                    if (geometry && !geometry.attributes.normal) {
                        geometry.computeVertexNormals();
                    }
                    
                    resolve(geometry);
                } catch (error) {
                    reject(error);
                }
            };
            
            reader.onerror = reject;
            
            // Read as ArrayBuffer for binary formats, Text for others
            if (extension === 'stl') { // STLLoader can handle both, but ArrayBuffer is common for binary
                reader.readAsArrayBuffer(file);
            } else if (extension === 'dae') { // ColladaLoader requires text
                 reader.readAsText(file);
            } else { // OBJLoader requires text
                reader.readAsText(file);
            }
        });
    }

    parseGeometry(geometryElement) {
        const box = geometryElement.getElementsByTagName('box')[0];
        if (box) {
            const size = box.getAttribute('size').split(' ').map(Number);
            return new THREE.BoxGeometry(size[0], size[1], size[2]);
        }
        
        const cylinder = geometryElement.getElementsByTagName('cylinder')[0];
        if (cylinder) {
            const radius = parseFloat(cylinder.getAttribute('radius'));
            const length = parseFloat(cylinder.getAttribute('length'));
            // Cylinder in Three.js is aligned with Y-axis, URDF typically Z-axis.
            // Rotate to align with Z-axis if necessary (visual only, not part of joint transform)
            const geometry = new THREE.CylinderGeometry(radius, radius, length, 32);
            geometry.rotateX(Math.PI / 2); // Rotate to align with Z-axis
            return geometry;
        }
        
        const sphere = geometryElement.getElementsByTagName('sphere')[0];
        if (sphere) {
            const radius = parseFloat(sphere.getAttribute('radius'));
            return new THREE.SphereGeometry(radius, 32, 32);
        }
        
        console.warn("Unknown primitive geometry type. Using fallback cube.");
        return new THREE.BoxGeometry(0.1, 0.1, 0.1);
    }

    parseMaterial(materialElement) {
        const material = new THREE.MeshStandardMaterial({
            color: 0x888888, // Default grey
            metalness: 0.2,
            roughness: 0.7,
            envMapIntensity: 0.9 // For more realistic PBR materials
        });
        
        if (materialElement) {
            const colorElement = materialElement.getElementsByTagName('color')[0];
            if (colorElement) {
                const rgba = colorElement.getAttribute('rgba').split(' ').map(Number);
                material.color.setRGB(rgba[0], rgba[1], rgba[2]);
                if (rgba[3] < 1) {
                    material.transparent = true;
                    material.opacity = rgba[3];
                }
            }
            // TODO: Handle texture element if needed
        }
        
        return material;
    }

    parseJoint(jointElement) {
        const name = jointElement.getAttribute('name');
        const type = jointElement.getAttribute('type');
        const parent = jointElement.getElementsByTagName('parent')[0]?.getAttribute('link');
        const child = jointElement.getElementsByTagName('child')[0]?.getAttribute('link');
        
        if (!parent || !child) {
            console.warn(`Joint ${name} is missing parent or child link.`);
            return;
        }
        
        const joint = {
            name: name,
            type: type,
            parent: parent,
            child: child,
            currentValue: 0,
            origin: { // This origin object will store the parsed values, not the DOM element
                xyz: [0, 0, 0],
                rpy: [0, 0, 0]
            }
        };
        
        // Parse origin
        const originElement = jointElement.getElementsByTagName('origin')[0]; // <--- Get the actual DOM element
        if (originElement) { // <--- Check if it exists
            if (originElement.hasAttribute('xyz')) {
                joint.origin.xyz = originElement.getAttribute('xyz').split(' ').map(Number);
            }
            if (originElement.hasAttribute('rpy')) {
                joint.origin.rpy = originElement.getAttribute('rpy').split(' ').map(Number);
            }
        }
        
        // Parse axis
        const axis = jointElement.getElementsByTagName('axis')[0];
        if (axis) {
            joint.axis = axis.getAttribute('xyz').split(' ').map(Number);
        } else {
            joint.axis = [1, 0, 0]; // Default axis
        }
        
        // Parse limits
        const limit = jointElement.getElementsByTagName('limit')[0];
        if (limit) {
            joint.lower = parseFloat(limit.getAttribute('lower') || 0);
            joint.upper = parseFloat(limit.getAttribute('upper') || 0);
            // Adjust for continuous joints if limits are not strictly defined
            if (type === 'continuous') {
                joint.lower = -Math.PI * 2; // Arbitrarily large range
                joint.upper = Math.PI * 2;
            }
        } else {
            // Default limits for revolute/prismatic if not specified
            if (type === 'revolute') {
                joint.lower = -Math.PI;
                joint.upper = Math.PI;
            } else if (type === 'prismatic') {
                joint.lower = -0.5; // Arbitrary small range
                joint.upper = 0.5;
            }
        }
        
        this.joints[name] = joint;
    }

    applyOrigin(object, originData) {
        let xyz = [0, 0, 0];
        let rpy = [0, 0, 0];
    
        // Check if originData is a DOM element (as it would be for visual/collision origins)
        if (originData instanceof Element) {
            if (originData.hasAttribute('xyz')) {
                xyz = originData.getAttribute('xyz').split(' ').map(Number);
            }
            if (originData.hasAttribute('rpy')) {
                rpy = originData.getAttribute('rpy').split(' ').map(Number);
            }
        } 
        // Check if originData is a parsed object (as it would be for joint origins)
        else if (originData && typeof originData === 'object' && Array.isArray(originData.xyz) && Array.isArray(originData.rpy)) {
            xyz = originData.xyz;
            rpy = originData.rpy;
        } else {
            // No valid origin data provided, use default (no transformation)
            object.position.set(0, 0, 0);
            object.rotation.set(0, 0, 0, 'ZYX'); // Set default rotation with correct Euler order
            return;
        }
        
        object.position.set(xyz[0], xyz[1], xyz[2]);
        object.rotation.order = 'ZYX'; // Set Euler order to ZYX for URDF compatibility
        object.rotation.set(rpy[0], rpy[1], rpy[2]);
    }

    createJointControls() {
        const controlsDiv = document.getElementById('jointControls');
        controlsDiv.innerHTML = ''; // Clear previous controls
        
        Object.keys(this.joints).forEach(jointName => {
            const joint = this.joints[jointName];
            
            if (joint.type === 'revolute' || joint.type === 'continuous' || joint.type === 'prismatic') {
                const controlDiv = document.createElement('div');
                controlDiv.className = 'joint-control';
                
                const label = document.createElement('label');
                label.textContent = jointName;
                
                const slider = document.createElement('input');
                slider.type = 'range';
                slider.min = joint.lower;
                slider.max = joint.upper;
                slider.step = (joint.upper - joint.lower) / 1000; // More fine-grained steps
                slider.value = 0; // Initial value
                
                const valueDisplay = document.createElement('span');
                valueDisplay.className = 'value';
                valueDisplay.textContent = '0.00';
                
                slider.addEventListener('input', (e) => {
                    const value = parseFloat(e.target.value);
                    this.updateJoint(jointName, value);
                    valueDisplay.textContent = value.toFixed(2);
                });
                
                controlDiv.appendChild(label);
                controlDiv.appendChild(slider);
                controlDiv.appendChild(valueDisplay);
                controlsDiv.appendChild(controlDiv);
                
                this.jointControls[jointName] = slider;
                this.updateJoint(jointName, 0); // Initialize joint position
            }
        });
    }

    updateJoint(jointName, value) {
        const joint = this.joints[jointName];
        if (!joint || !this.links[joint.child]) return;
        
        const childLink = this.links[joint.child];
        
        // Reset to initial position/rotation relative to parent (due to joint origin)
        childLink.position.copy(joint.initialPosition);
        childLink.quaternion.copy(joint.initialRotation);

        if (joint.type === 'revolute' || joint.type === 'continuous') {
            const axis = new THREE.Vector3(...joint.axis).normalize();
            // childLink.rotateOnWorldAxis(axis, value); // Rotate around the joint's axis

            const axisVector = new THREE.Vector3(...joint.axis).normalize();
            const quaternion = new THREE.Quaternion().setFromAxisAngle(axisVector, value);
            childLink.quaternion.multiplyQuaternions(joint.initialRotation, quaternion);

        } else if (joint.type === 'prismatic') {
            const axis = new THREE.Vector3(...joint.axis).normalize();
            // Prismatic joint translates along its axis
            const translationVector = axis.multiplyScalar(value);
            childLink.position.add(translationVector);
        }
        
        joint.currentValue = value;
    }

    toggleAxes(show) {
        this.axesHelpers.forEach(helper => {
            helper.visible = show;
        });
    }

    toggleWireframe(wireframe) {
        if (this.robot) {
            this.robot.traverse(child => {
                if (child.isMesh && child.material) {
                    if (Array.isArray(child.material)) {
                        child.material.forEach(mat => {
                            if (mat.isMaterial) mat.wireframe = wireframe;
                        });
                    } else if (child.material.isMaterial) {
                        child.material.wireframe = wireframe;
                    }
                }
            });
        }
    }

    toggleGrid(show) {
        if (this.gridHelper) {
            this.gridHelper.visible = show;
        }
    }

    fitCameraToObject() {
        if (!this.robot || !this.robot.children.length) {
            this.resetCamera();
            return;
        }
        
        const box = new THREE.Box3().setFromObject(this.robot);
        if (box.isEmpty()) {
            this.resetCamera();
            return;
        }

        const size = box.getSize(new THREE.Vector3());
        const center = box.getCenter(new THREE.Vector3());
        
        const maxDim = Math.max(size.x, size.y, size.z);
        const fov = this.camera.fov * (Math.PI / 180);
        let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
        
        // Add some padding
        cameraZ *= 1.5; 
        
        this.camera.position.set(center.x + cameraZ, center.y + cameraZ, center.z + cameraZ);
        this.camera.lookAt(center);
    }

    updateStatus(message, type) {
        const statusDiv = document.getElementById('status');
        statusDiv.textContent = message;
        statusDiv.className = `status-${type}`;
    }

    animate() {
        requestAnimationFrame(() => this.animate());

        if (this.autoRotateEnabled) {
            this.robot.rotation.z += 0.005; // Gentle rotation
        }

        // This line is crucial for rendering the 3D scene
        this.renderer.render(this.scene, this.camera);
    }
}




// Ensure the ProfessionalURDFViewer is instantiated, typically at the end of the file
window.addEventListener('DOMContentLoaded', () => {
    window.viewer = new ProfessionalURDFViewer();
});

// Handle window resize
window.addEventListener('resize', () => {
    if (window.viewer) {
        window.viewer.camera.aspect = (window.innerWidth - 350) / window.innerHeight; // Adjust for sidebar
        window.viewer.camera.updateProjectionMatrix();
        window.viewer.renderer.setSize(window.innerWidth - 350, window.innerHeight); // Adjust for sidebar
    }
});
