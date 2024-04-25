import * as THREE from 'three';
import * as THREEv140 from 'three-v140';
import { FontLoader } from 'three/addons/loaders/FontLoader.js';
import { TextGeometry } from 'three/addons/geometries/TextGeometry.js';
import GraphBar from 'GraphBar';

let scene = null;
let camera = null;
let renderer = null;
let airQBar = null;
let robotoFont = null;
let currentValue = 0;
let minValue = 0;
let maxValue = 0;
let smogParts = [];
let carsFullImage = null;
let carsHalfFullImage = null;

const clockContainer = document.getElementById('js-digital-clock');

const getDateSuffix = date => {

    if (date > 3 && date < 21) {
        return 'th';
    }

    switch (date % 10) {
        case 1:
            return 'st';
        case 2:
            return 'nd';
        case 3:
            return 'rd';
    }
};

const processData = apiData => {

    apiData.forEach(apiDatum => {
        if (apiDatum['Average'] < minValue) {
            minValue = apiDatum['Average'];
        }
        if (apiDatum['Average'] > maxValue) {
            maxValue = apiDatum['Average'];
        }
    });

    const yesterday = new Date(apiData[0].sensorReadingDate.split(' ')[0]);
    const dateContainer = document.getElementById('js-date');
    dateContainer.textContent = `${yesterday.getDate()}${getDateSuffix(yesterday.getDate())} ${yesterday.toLocaleString('en-GB', { month: 'short' })}, ${yesterday.getFullYear()}`;

    animate(apiData);
}

const animate = (apiData) => {

    let seconds = {
        current: 0,
        max: 86400
    }

    gsap.to(seconds, {
        current: seconds.max,
        duration: 60,
        ease: 'none',
        onUpdate: function () {
            const percentage = seconds.current / seconds.max;
            const index = Math.round((apiData.length - 1) * percentage);

            if (seconds.current > 86399) {
                // Stop the clock @ 23:59
                seconds.current = 86399;
            }

            let hours = Math.floor(seconds.current / 3600);
            let minutes = Math.floor(seconds.current % 3600 / 60);
            if (minutes < 10) {
                minutes = `0${minutes}`;
            }

            let time = `${hours}:${minutes}am`
            if (hours == 0) {
                time = `12:${minutes}am`
            } else if (hours == 12) {
                time = `12:${minutes}pm`
            } else if (hours > 12) {
                time = `${hours - 12}:${minutes}pm`
            }
            clockContainer.textContent = time;

            updateCarsIllustration(hours);

            if (currentValue != apiData[index]['Average']) {

                currentValue = apiData[index]['Average'];

                updateIllustration();
                airQBar.update(currentValue);
            }
        },
        onComplete: function () {
            setTimeout(function () {
                animate(apiData);
            }, 10000);
        }
    });
}

const updateIllustration = () => {

    const range = maxValue - minValue;
    const percent = currentValue / range;

    const maxSmogPartsIndex = Math.round((smogParts.length - 1) * percent);

    for (let i = 0; i < smogParts.length - 1; i++) {
        const smogPart = smogParts[i];

        if (i <= maxSmogPartsIndex) {
            smogPart.classList.remove('opacity-0');
            smogPart.classList.add('opacity-100');
        } else {
            smogPart.classList.add('opacity-0');
            smogPart.classList.remove('opacity-100');
        }
    }
}

const updateCarsIllustration = (hourOfTheDay) => {

    if (hourOfTheDay >= 17 || hourOfTheDay < 8) {
        carsFullImage.classList.add('opacity-0');
        carsFullImage.classList.remove('opacity-100');
        carsHalfFullImage.classList.add('opacity-0');
        carsHalfFullImage.classList.remove('opacity-100');
    } else if (hourOfTheDay < 12 || hourOfTheDay >= 14) {
        carsFullImage.classList.add('opacity-100');
        carsFullImage.classList.remove('opacity-0');
        carsHalfFullImage.classList.add('opacity-0');
        carsHalfFullImage.classList.remove('opacity-100');
    } else {
        carsFullImage.classList.add('opacity-0');
        carsFullImage.classList.remove('opacity-100');
        carsHalfFullImage.classList.add('opacity-100');
        carsHalfFullImage.classList.remove('opacity-0');
    }
}

const fontLoaderLoadHandler = font => {

    robotoFont = font;

    let material = new THREE.MeshPhongMaterial({
        color: 0x7F7F7F
    });

    let geometry = new TextGeometry(`Air Quality`, {
        font: font,
        size: .07,
        height: .009
    });
    geometry.computeBoundingBox();
    geometry.computeVertexNormals();

    const titleMesh = new THREE.Mesh(geometry, material);
    titleMesh.position.x = -2.2;
    titleMesh.position.y = -1.35;
    titleMesh.position.z = -1.145;
    titleMesh.rotation.x = 0;
    titleMesh.rotation.y = 1.57;
    titleMesh.rotation.z = 0;
    scene.add(titleMesh);

    robotoFont = font;

    material = new THREE.MeshPhongMaterial({
        color: 0x7F7F7F
    });

    geometry = new TextGeometry(`(PM2.5)`, {
        font: font,
        size: .035,
        height: .0045
    });
    geometry.computeBoundingBox();
    geometry.computeVertexNormals();

    const pm2Mesh = new THREE.Mesh(geometry, material);
    pm2Mesh.position.x = -2.2;
    pm2Mesh.position.y = -1.42;
    pm2Mesh.position.z = -1.15;
    pm2Mesh.rotation.x = 0;
    pm2Mesh.rotation.y = 1.57;
    pm2Mesh.rotation.z = 0;
    scene.add(pm2Mesh);

    material = new THREE.MeshPhongMaterial({
        color: 0xffffff
    });

    airQBar = new GraphBar(scene, 'Air Quality');
    airQBar.width = .71;
    airQBar.height = 0.15;
    airQBar.thickness = 0.0150;
    airQBar.cornerRadius = 0.025;
    airQBar.smoothness = 36;
    airQBar.x = -2.23;
    airQBar.y = -1.66;
    airQBar.z = -1.185;
    airQBar.rotationX = 0;
    airQBar.rotationY = 1.57;
    airQBar.rotationZ = 0;
    airQBar.update(0);

    getData();
}

const getData = () => {

    Date.prototype.yyyymmdd = function () {
        const mm = this.getMonth() + 1; // getMonth() is zero-based
        const dd = this.getDate() - 1;

        return [this.getFullYear(),
        (mm > 9 ? '' : '0') + mm,
        (dd > 9 ? '' : '0') + dd
        ].join('-');
    };

    const date = new Date();
    const yesterday = date.yyyymmdd();
    const startDate = yesterday + ' 00:00:00';
    const endDate = yesterday + ' 23:59:59';

    fetch('https://api.core.aql.com/v1/sensors/sensor-data/aggregate/ave', {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Authorization': 'Bearer 11371|4L6TMzBwVlykgcc9nASWJTNriPeysVJhOde57XIO',
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            'sensor_ids': [
                'ynAjin2RKZ6'
            ],
            'startDate': startDate,
            'endDate': endDate,
            'sampleInterval': 1,
            'granularity': 'five_minutes',
        })
    }).then(response => {
        if (response.status === 200) {
            response.json().then(data => processData(data));
        }
    });
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

    const smogWrapper = document.querySelector('.js-smog-wrapper');
    const smogPartsWrapper = document.createElement('div');
    smogPartsWrapper.classList = 'relative';
    smogWrapper.append(smogPartsWrapper);

    for (let i = 1; i < 14; i++) {
        const smogPart = document.createElement('img');
        smogPart.src = `images/illustrations/smog-${i}.svg`;
        smogPart.classList = `js-smog-part-${i} absolute scale-[0.4] opacity-0 transition-all duration-500`;
        smogPartsWrapper.append(smogPart);
        smogParts.push(smogPart);
    }

    carsFullImage = document.querySelector('.js-cars-image-full');
    carsHalfFullImage = document.querySelector('.js-cars-image-half-full');

    const fontLoader = new FontLoader();
    fontLoader.load('fonts/roboto-bold.typeface.json', fontLoaderLoadHandler);
}

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