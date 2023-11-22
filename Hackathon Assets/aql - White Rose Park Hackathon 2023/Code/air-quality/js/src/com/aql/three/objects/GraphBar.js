import * as THREE from 'three';
import { FontLoader } from 'three/addons/loaders/FontLoader.js';
import { TextGeometry } from 'three/addons/geometries/TextGeometry.js';
import { DecalGeometry } from 'three/addons/geometries/DecalGeometry.js';

class GraphBar {

    currentPercentage = 0;
    maxWidth = .71;
    minWidth = .15;

    constructor(scene, name) {
        this.scene = scene;
        this.name = name;
        this.width = 1;
        this.height = 1;
        this.thickness = 0.02;
        this.cornerRadius = 0.025;
        this.smoothness = 36;
        this.x = 0;
        this.y = 0;
        this.rotationX = 0;
        this.rotationY = 0;
        this.rotationZ = 0;

        const material = new THREE.MeshPhongMaterial({
            color: 0x049ef4,
        });

        this.mesh = new THREE.Mesh(this.geometry, material);
        this.scene.add(this.mesh);

        const texture = new THREE.TextureLoader().load('images/air-qual-icon.png');

        const decalMaterial = new THREE.MeshPhongMaterial({
            map: texture,
            normalMap: texture,
            normalScale: new THREE.Vector2(5, 5),
            transparent: true,
            depthTest: true,
            depthWrite: false,
            polygonOffset: true,
            polygonOffsetFactor: -4,
            wireframe: false,
            side: THREE.FrontSide
        });

        this.decal = new THREE.Mesh(this.decalGeometry, decalMaterial);
        this.scene.add(this.decal);

        const loader = new FontLoader();
        loader.load('fonts/roboto-bold.typeface.json', this.fontLoaderLoadHandler.bind(this));
    }

    update(value) {

        this.value = value;

        if (value >= 10) {
            this.mesh.material.color.setHex(0xBA0F30);
        } else if (value > 4) {
            this.mesh.material.color.setHex(0xFFC300);
        } else {
            this.mesh.material.color.setHex(0x50C878);
        }

        this.mesh.geometry.dispose();
        this.mesh.geometry = this.geometry;
        this.mesh.position.x = this.x;
        this.mesh.position.y = this.y;
        this.mesh.position.z = this.z;
        this.mesh.rotation.x = this.rotationX;
        this.mesh.rotation.y = this.rotationY;
        this.mesh.rotation.z = this.rotationZ;

        this.decal.geometry.dispose();
        this.decal.geometry = this.decalGeometry;

        if (this.percentageMesh) {
            this.percentageMesh.geometry.dispose();
            this.percentageMesh.geometry = this.percentageGeometry;
            this.percentageMesh.position.z = this.z - (this.width - (this.percentageMesh.geometry.boundingBox.max.x + .04));
        }
    }

    fontLoaderLoadHandler(font) {

        this.font = font;

        const material = new THREE.MeshPhongMaterial({
            color: 0xffffff
        });

        this.percentageMesh = new THREE.Mesh(this.percentageGeometry, material);
        this.percentageMesh.position.x = this.x + .005;
        this.percentageMesh.position.y = this.y + .05;
        this.percentageMesh.position.z = (this.width - (this.percentageMesh.geometry.boundingBox.max.x + .04));

        this.percentageMesh.rotation.x = this.rotationX;
        this.percentageMesh.rotation.y = this.rotationY;
        this.percentageMesh.rotation.z = this.rotationZ;
        this.scene.add(this.percentageMesh);
    }

    get geometry() {
        const geometry = this.getRoundedRect();
        geometry.computeVertexNormals();
        return geometry;
    }

    get percentageGeometry() {
        const geometry = new TextGeometry(`${Math.round(this.value)}µg/m3`, {
            font: this.font,
            size: .07,
            height: .01
        });
        geometry.computeBoundingBox();
        geometry.computeVertexNormals();
        return geometry;
    }

    get decalGeometry() {
        return new DecalGeometry(
            this.mesh,
            new THREE.Vector3(
                this.mesh.position.x,
                this.mesh.position.y + .075,
                this.mesh.position.z - 0.08
            ),
            new THREE.Euler(
                this.mesh.rotation.x,
                this.mesh.rotation.y,
                this.mesh.rotation.z
            ),
            new THREE.Vector3(.08, .08, 1)
        );
    }

    get x() {
        return this._x;
    }

    set x(x) {
        this._x = x;
    }

    get y() {
        return this._y;
    }

    set y(y) {
        this._y = y;
    }

    get z() {
        return this._z;
    }

    set z(z) {
        this._z = z;
    }

    get rotationX() {
        return this._rotationX;
    }

    set rotationX(rotationX) {
        this._rotationX = rotationX;
    }

    get rotationY() {
        return this._rotationY;
    }

    set rotationY(rotationY) {
        this._rotationY = rotationY;
    }

    get rotationZ() {
        return this._rotationZ;
    }

    set rotationZ(rotationZ) {
        this._rotationZ = rotationZ;
    }

    getRoundedRect() {

        const wi = this.width / 2 - this.cornerRadius;
        const hi = this.height / 2 - this.cornerRadius;
        const w2 = this.width / 2;
        const h2 = this.height / 2;

        let ul = this.cornerRadius / this.width;
        let ur = (this.width - this.cornerRadius) / this.width;
        const vl = this.cornerRadius / this.height;
        const vh = (this.height - this.cornerRadius) / this.height;

        let phia, phib, xc, yc, uc, vc, cosa, sina, cosb, sinb;
        let positions = [];
        let uvs = [];
        let t2 = this.thickness / 2;
        let u0 = ul;
        let u1 = ur;
        let u2 = 0;
        let u3 = 1;
        let sign = 1;

        for (let k = 0; k < 2; k++) {

            positions.push(
                -wi, -h2, t2, wi, -h2, t2, wi, h2, t2,
                -wi, -h2, t2, wi, h2, t2, -wi, h2, t2,
                -w2, -hi, t2, -wi, -hi, t2, -wi, hi, t2,
                -w2, -hi, t2, -wi, hi, t2, -w2, hi, t2,
                wi, -hi, t2, w2, -hi, t2, w2, hi, t2,
                wi, -hi, t2, w2, hi, t2, wi, hi, t2
            );

            uvs.push(
                u0, 0, u1, 0, u1, 1,
                u0, 0, u1, 1, u0, 1,
                u2, vl, u0, vl, u0, vh,
                u2, vl, u0, vh, u2, vh,
                u1, vl, u3, vl, u3, vh,
                u1, vl, u3, vh, u1, vh
            );

            phia = 0;

            for (let i = 0; i < this.smoothness * 4; i++) {

                phib = Math.PI * 2 * (i + 1) / (4 * this.smoothness);
                cosa = Math.cos(phia);
                sina = Math.sin(phia);
                cosb = Math.cos(phib);
                sinb = Math.sin(phib);

                xc = i < this.smoothness || i >= 3 * this.smoothness ? wi : -wi;
                yc = i < 2 * this.smoothness ? hi : -hi;
                positions.push(xc, yc, t2, xc + this.cornerRadius * cosa, yc + this.cornerRadius * sina, t2, xc + this.cornerRadius * cosb, yc + this.cornerRadius * sinb, t2);
                uc = i < this.smoothness || i >= 3 * this.smoothness ? u1 : u0;
                vc = i < 2 * this.smoothness ? vh : vl;

                uvs.push(uc, vc, uc + sign * ul * cosa, vc + vl * sina, uc + sign * ul * cosb, vc + vl * sinb);

                phia = phib;
            }

            t2 = -t2;
            u0 = ur;
            u1 = ul;
            u2 = 1;
            u3 = 0;
            sign = -1;
        }

        t2 = this.thickness / 2;

        positions.push(
            -wi, -h2, t2, -wi, -h2, -t2, wi, -h2, -t2,
            -wi, -h2, t2, wi, -h2, -t2, wi, -h2, t2,
            w2, -hi, t2, w2, -hi, -t2, w2, hi, -t2,
            w2, -hi, t2, w2, hi, -t2, w2, hi, t2,
            wi, h2, t2, wi, h2, -t2, -wi, h2, -t2,
            wi, h2, t2, -wi, h2, -t2, -wi, h2, t2,
            -w2, hi, t2, -w2, hi, -t2, -w2, -hi, -t2,
            -w2, hi, t2, -w2, -hi, -t2, -w2, -hi, t2
        );

        const cf = 2 * ((this.width + this.height - 4 * this.cornerRadius) + Math.PI * this.cornerRadius);
        const cc4 = Math.PI * this.cornerRadius / 2 / cf;
        u0 = 0;
        u1 = 2 * wi / cf;
        u2 = u1 + cc4;
        u3 = u2 + 2 * hi / cf;

        const u4 = u3 + cc4;
        const u5 = u4 + 2 * wi / cf;
        const u6 = u5 + cc4;
        const u7 = u6 + 2 * hi / cf;

        uvs.push(
            u0, 1, 0, 0, u1, 0,
            u0, 1, u1, 0, u1, 1,
            u2, 1, u2, 0, u3, 0,
            u2, 1, u3, 0, u3, 1,
            u4, 1, u4, 0, u5, 0,
            u4, 1, u5, 0, u5, 1,
            u6, 1, u6, 0, u7, 0,
            u6, 1, u7, 0, u7, 1
        );

        phia = 0;

        let u, j, j1;
        const ccs = cc4 / this.smoothness;

        for (let i = 0; i < this.smoothness * 4; i++) {

            phib = Math.PI * 2 * (i + 1) / (4 * this.smoothness);

            cosa = Math.cos(phia);
            sina = Math.sin(phia);
            cosb = Math.cos(phib);
            sinb = Math.sin(phib);

            xc = i < this.smoothness || i >= 3 * this.smoothness ? wi : -wi;
            yc = i < 2 * this.smoothness ? hi : -hi;

            positions.push(xc + this.cornerRadius * cosa, yc + this.cornerRadius * sina, t2, xc + this.cornerRadius * cosa, yc + this.cornerRadius * sina, -t2, xc + this.cornerRadius * cosb, yc + this.cornerRadius * sinb, -t2);
            positions.push(xc + this.cornerRadius * cosa, yc + this.cornerRadius * sina, t2, xc + this.cornerRadius * cosb, yc + this.cornerRadius * sinb, -t2, xc + this.cornerRadius * cosb, yc + this.cornerRadius * sinb, t2);

            u = i < this.smoothness ? u3 : (i < 2 * this.smoothness ? u5 : (i < 3 * this.smoothness ? u7 : u1));

            j = i % this.smoothness;
            j1 = j + 1;

            uvs.push(u + j * ccs, 1, u + j * ccs, 0, u + j1 * ccs, 0);
            uvs.push(u + j * ccs, 1, u + j1 * ccs, 0, u + j1 * ccs, 1);

            phia = phib;
        }

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
        geometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uvs), 2));

        const vtc = (6 + 4 * this.smoothness) * 3;
        geometry.addGroup(0, vtc, 0);
        geometry.addGroup(vtc, vtc, 1);
        geometry.addGroup(2 * vtc, 24 + 2 * 3 * 4 * this.smoothness, 2);

        geometry.translate(w2, h2, 0)

        return geometry;
    }
}

export default GraphBar;