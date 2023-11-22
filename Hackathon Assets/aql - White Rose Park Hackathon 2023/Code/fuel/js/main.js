import * as THREE from 'three';
import * as THREEv140 from 'three-v140';
import GraphBar from 'GraphBar';
import KeyControl from 'KeyControl';
import { FontLoader } from 'three/addons/loaders/FontLoader.js';
import { TextGeometry } from 'three/addons/geometries/TextGeometry.js';

let scene = null;
let camera = null;
let renderer = null;
let fuelTank1Bar = null;
let fuelTank2Bar = null;

const addGraphBars = apiData => {

    fuelTank1Bar = new GraphBar(scene, 'Fuel Tank 1 Bar');
    fuelTank1Bar.width = .1;
    fuelTank1Bar.height = 0.15;
    fuelTank1Bar.thickness = 0.0150;
    fuelTank1Bar.cornerRadius = 0.025;
    fuelTank1Bar.smoothness = 36;
    fuelTank1Bar.x = -1.925;
    fuelTank1Bar.y = -1.51;
    fuelTank1Bar.z = -1.185;
    fuelTank1Bar.rotationX = 0;
    fuelTank1Bar.rotationY = 1.59;
    fuelTank1Bar.rotationZ = 0;
    fuelTank1Bar.update();

    fuelTank2Bar = new GraphBar(scene, 'Fuel Tank 2 Bar');
    fuelTank2Bar.width = .1;
    fuelTank2Bar.height = 0.15;
    fuelTank2Bar.thickness = 0.0150;
    fuelTank2Bar.cornerRadius = 0.025;
    fuelTank2Bar.smoothness = 36;
    fuelTank2Bar.x = -1.925;
    fuelTank2Bar.y = -1.32;
    fuelTank2Bar.z = -1.185;
    fuelTank2Bar.rotationX = 0;
    fuelTank2Bar.rotationY = 1.59;
    fuelTank2Bar.rotationZ = 0;
    fuelTank2Bar.update();

    fuelTank1Bar.animateToPercentage(apiData[0]['value']);
    fuelTank2Bar.animateToPercentage(apiData[1]['value']);

    // Assign object for keyboard control
    // KeyControl.threeDObject = fuelTank1Bar;

    setUpdateTimeout();
}

const setUpdateTimeout = () => {

    setTimeout(() => {
        fetchData(updateGraphBars);
    }, 30000);
};

const updateGraphBars = apiData => {

    fuelTank1Bar.animateToPercentage(apiData[0]['value']);
    fuelTank2Bar.animateToPercentage(apiData[1]['value']);

    setUpdateTimeout();
};

const fontLoaderLoadHandler = font => {

    const material = new THREE.MeshPhongMaterial({
        color: 0x7F7F7F
    });

    const geometry = new TextGeometry(`Fuel Level`, {
        font: font,
        size: .1,
        height: .009
    });
    geometry.computeBoundingBox();
    geometry.computeVertexNormals();

    const titleMesh = new THREE.Mesh(geometry, material);
    titleMesh.position.x = -1.925;
    titleMesh.position.y = -1.1;
    titleMesh.position.z = -1.165;
    titleMesh.rotation.x = 0;
    titleMesh.rotation.y = 1.59;
    titleMesh.rotation.z = 0;
    scene.add(titleMesh);;
}

const render = () => {
    requestAnimationFrame(render);
    renderer.render(scene, camera);
}

const domContentLoadedHandler = event => {

    initHeaderBackground();

    // Scene setup
    const sceneWrapper = document.getElementById('js-scene-wrapper');
    const aspect = sceneWrapper.clientWidth / sceneWrapper.clientHeight;
    scene = new THREE.Scene();

    // Lights (directional and ambient)
    const light = new THREE.DirectionalLight();
    light.position.set(50, 200, 100);
    light.position.multiplyScalar(1.3);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0xffffff));

    // Camera (isometric scene settings)
    camera = new THREE.OrthographicCamera(-1 * aspect, aspect, 1, -1, 1, 1000);
    camera.position.set(20, 20, 20);
    camera.lookAt(scene.position);

    // Action
    const canvas = document.getElementById('js-scene-canvas');
    renderer = new THREE.WebGLRenderer({
        antialias: true,
        alpha: true,
        canvas: canvas,
    });
    renderer.toneMapping = THREE.LinearToneMapping;
    renderer.toneMappingExposure = 1.6;
    renderer.setPixelRatio(2);
    renderer.setSize(sceneWrapper.clientWidth, sceneWrapper.clientHeight);
    sceneWrapper.append(renderer.domElement);

    render();

    const fontLoader = new FontLoader();
    fontLoader.load('fonts/roboto-bold.typeface.json', fontLoaderLoadHandler);

    fetchData(addGraphBars);
}

const fetchData = completeHandler => {

    fetch('https://api.core.aql.com/v1/sensors/sensor-data/latest', {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Authorization': 'Bearer 11371|4L6TMzBwVlykgcc9nASWJTNriPeysVJhOde57XIO',
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            'sensor_ids': [
                'wy00upVWkK0',
                '87mPCrPKW1O'
            ]
        })
    }).then(response => {
        if (response.status === 200) {
            response.json().then(data => completeHandler(data));
        }
    });
};

const initHeaderBackground = () => {

    const canvasWrapper = document.getElementById('js-header-canvas');

    if (!canvasWrapper || window.innerWidth < 768) {
        return;
    }

    VANTA.NET({
        el: canvasWrapper,
        mouseControls: true,
        touchControls: false,
        gyroControls: false,
        scale: 1,
        color: '#39A3D1',
        backgroundColor: '#2869B0',
        points: 10,
        spacing: 15,
        maxDistance: 30,
        THREE: THREEv140,
        minHeight: 1000,
        minWidth: 1000,
        mouseCoeffX: .05,
        mouseCoeffY: .05
    });
};

document.addEventListener('DOMContentLoaded', domContentLoadedHandler);