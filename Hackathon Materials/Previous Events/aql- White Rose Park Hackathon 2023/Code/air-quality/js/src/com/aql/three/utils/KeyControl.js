class KeyControl {

    modifer = 'w';
    increment = .1;

    constructor() {
        if (!KeyControl.instance) {
            KeyControl.instance = this;
            this._threeDObject = null;
            window.addEventListener('keydown', this.keyDownHandler.bind(this));
        }
        return KeyControl.instance;
    }

    keyDownHandler(event) {
        if (!this.threeDObject) {
            return;
        }

        let keys = ['ArrowUp', 'ArrowDown'];
        if (keys.includes(event.key)) {
            event.preventDefault();
        }

        keys = ['w', 'h', 't', 'c', 's', 'p', 'r', 'i'];
        if (keys.includes(event.key)) {
            this.modifer = event.key;
            return;
        }

        if (event.code === 'Space') {
            event.preventDefault();
            console.log(`${this.threeDObject.name} properties:`,
                `\n`,
                `width: ${this.threeDObject.width}`,
                `\n`,
                `height: ${this.threeDObject.height}`,
                `\n`,
                `thickness: ${this.threeDObject.thickness}`,
                `\n`,
                `cornerRadius: ${this.threeDObject.cornerRadius}`,
                `\n`,
                `smoothness: ${this.threeDObject.smoothness}`,
                `\n`,
                `x: ${this.threeDObject.mesh.position.x}`,
                `\n`,
                `y: ${this.threeDObject.mesh.position.y}`,
                `\n`,
                `z: ${this.threeDObject.mesh.position.z}`,
                `\n`,
                `rotationX: ${this.threeDObject.mesh.rotation.x}`,
                `\n`,
                `rotationY: ${this.threeDObject.mesh.rotation.y}`,
                `\n`,
                `rotationZ: ${this.threeDObject.mesh.rotation.z}`,
            );
            return;
        }

        if (this.modifer === 'i') {

            if (event.key === 'ArrowUp') {
                this.increment += .0025;
            } else if (event.key === 'ArrowDown') {
                this.increment -= .0025;
            }

            console.log(`Increment: ${this.increment}`);

        } else if (this.modifer === 'w') {

            if (event.key === 'ArrowUp') {
                this.threeDObject.width += this.increment;
            } else if (event.key === 'ArrowDown') {
                this.threeDObject.width -= this.increment;
            }

            this.threeDObject.update();

        } else if (this.modifer === 'h') {

            if (event.key === 'ArrowUp') {
                this.threeDObject.height += this.increment;
            } else if (event.key === 'ArrowDown') {
                this.threeDObject.height -= this.increment;
            }

            this.threeDObject.update();

        } else if (this.modifer === 'p') {

            if (event.shiftKey) {

                if (event.key === 'ArrowUp') {
                    this.threeDObject.mesh.position.z += this.increment;
                } else if (event.key === 'ArrowDown') {
                    this.threeDObject.mesh.position.z -= this.increment;
                }

            } else {

                if (event.key === 'ArrowLeft') {
                    this.threeDObject.mesh.position.x -= this.increment;
                } else if (event.key === 'ArrowRight') {
                    this.threeDObject.mesh.position.x += this.increment;
                } else if (event.key === 'ArrowUp') {
                    this.threeDObject.mesh.position.y += this.increment;
                } else if (event.key === 'ArrowDown') {
                    this.threeDObject.mesh.position.y -= this.increment;
                }
            }

        } else if (this.modifer === 'r') {

            if (event.shiftKey) {

                if (event.key === 'ArrowUp') {
                    this.threeDObject.mesh.rotation.x -= this.increment;
                } else if (event.key === 'ArrowDown') {
                    this.threeDObject.mesh.rotation.x += this.increment;
                }

            } else {

                if (event.key === 'ArrowLeft') {
                    this.threeDObject.mesh.rotation.z += this.increment;
                } else if (event.key === 'ArrowRight') {
                    this.threeDObject.mesh.rotation.z -= this.increment;
                } else if (event.key === 'ArrowUp') {
                    this.threeDObject.mesh.rotation.y -= this.increment;
                } else if (event.key === 'ArrowDown') {
                    this.threeDObject.mesh.rotation.y += this.increment;
                }
            }

        } else if (this.modifer === 't') {

            if (event.key === 'ArrowUp') {
                this.threeDObject.thickness += this.increment * .25;
            } else if (event.key === 'ArrowDown') {
                this.threeDObject.thickness -= this.increment * .25;
            }

            this.threeDObject.update();

        } else if (this.modifer === 'c') {

            if (event.key === 'ArrowUp') {
                this.threeDObject.cornerRadius += this.increment * .25;
            } else if (event.key === 'ArrowDown') {
                this.threeDObject.cornerRadius -= this.increment * .25;
            }

            this.threeDObject.update();

        } else if (this.modifer === 's') {

            if (event.key === 'ArrowUp') {
                this.threeDObject.smoothness += this.increment;
            } else if (event.key === 'ArrowDown') {
                this.threeDObject.smoothness -= this.increment;
            }

            this.threeDObject.update();
        }
    }

    set threeDObject(threeDObject) {
        this._threeDObject = threeDObject;
    }

    get threeDObject() {
        return this._threeDObject;
    }
}

export default new KeyControl();

// const increment = 0.1;
// let modifer = 'w';

// window.addEventListener('keydown', event => {
//     // console.log(event);

// const keys = ['w', 'h', 't', 'c', 's', 'p', 'r'];
// if (keys.includes(event.key)) {
//     modifer = event.key;
//     return;
// }

//     if (event.code === 'Space') {
//         console.log(`{
//             w: ${w},
//             h: ${h},
//             t: ${t},
//             r: ${r},
//             s: ${s},
//             x: ${fuelTank1Bar.position.x},
//             y: ${fuelTank1Bar.position.y},
//             z: ${fuelTank1Bar.position.z},
//             rotateX: ${fuelTank1Bar.rotation.x},
//             rotateY: ${fuelTank1Bar.rotation.y},
//             rotateZ: ${fuelTank1Bar.rotation.z},
//         }`);
//         return;
//     }

//     if (modifer === 'w') {

//         if (event.key === 'ArrowUp') {
//             w += increment;
//         } else if (event.key === 'ArrowDown') {
//             w -= increment;
//         }

//         geometry.dispose();
//         geometry = RoundEdgedBoxFlat(w, h, t, r, s);
//         geometry.computeVertexNormals();
//         fuelTank1Bar.geometry = geometry;

//     } else if (modifer === 'h') {

//         if (event.key === 'ArrowUp') {
//             h += increment;
//         } else if (event.key === 'ArrowDown') {
//             h -= increment;
//         }

//         geometry.dispose();
//         geometry = RoundEdgedBoxFlat(w, h, t, r, s);
//         geometry.computeVertexNormals();
//         fuelTank1Bar.geometry = geometry;

//     } else if (modifer === 'p') {

//         if (event.shiftKey) {

//             if (event.key === 'ArrowUp') {
//                 // setZ((fuelTank1Bar.position.z - (w / 2)) + increment);
//                 setZ((fuelTank1Bar.position.z + increment));
//                 // fuelTank1Bar.translateZ = w / 2;
//             } else if (event.key === 'ArrowDown') {
//                 setZ((fuelTank1Bar.position.z - increment));
//             }

//         } else {

//             if (event.key === 'ArrowLeft') {
//                 fuelTank1Bar.position.x -= increment;
//             } else if (event.key === 'ArrowRight') {
//                 fuelTank1Bar.position.x += increment;
//             } else if (event.key === 'ArrowUp') {
//                 fuelTank1Bar.position.y += increment;
//             } else if (event.key === 'ArrowDown') {
//                 fuelTank1Bar.position.y -= increment;
//             }
//         }

//     } else if (modifer === 'r') {

//         if (event.shiftKey) {

//             if (event.key === 'ArrowUp') {
//                 fuelTank1Bar.rotation.x -= increment;
//             } else if (event.key === 'ArrowDown') {
//                 fuelTank1Bar.rotation.x += increment;
//             }

//         } else {

//             if (event.key === 'ArrowLeft') {
//                 fuelTank1Bar.rotation.z += increment;
//             } else if (event.key === 'ArrowRight') {
//                 fuelTank1Bar.rotation.z -= increment;
//             } else if (event.key === 'ArrowUp') {
//                 fuelTank1Bar.rotation.y -= increment;
//             } else if (event.key === 'ArrowDown') {
//                 fuelTank1Bar.rotation.y += increment;
//             }
//         }

//     } else if (modifer === 't') {

//         if (event.key === 'ArrowUp') {
//             t += increment * .25;
//         } else if (event.key === 'ArrowDown') {
//             t -= increment * .25;
//         }

//         geometry.dispose();
//         geometry = RoundEdgedBoxFlat(w, h, t, r, s);
//         geometry.computeVertexNormals();
//         fuelTank1Bar.geometry = geometry;

//     } else if (modifer === 'c') {

//         if (event.key === 'ArrowUp') {
//             r += increment * .25;
//         } else if (event.key === 'ArrowDown') {
//             r -= increment * .25;
//         }

//         geometry.dispose();
//         geometry = RoundEdgedBoxFlat(w, h, t, r, s);
//         geometry.computeVertexNormals();
//         fuelTank1Bar.geometry = geometry;

//     } else if (modifer === 's') {

//         if (event.key === 'ArrowUp') {
//             s += increment;
//         } else if (event.key === 'ArrowDown') {
//             s -= increment;
//         }

//         geometry.dispose();
//         geometry = RoundEdgedBoxFlat(w, h, t, r, s);
//         geometry.computeVertexNormals();
//         fuelTank1Bar.geometry = geometry;
//     }
// });