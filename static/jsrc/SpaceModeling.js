class EciTrajectory extends Line {
    constructor(propagator, color = 0xFFFFFF, minutes = 720, stepSeconds = 180) {

        // params
        const numSteps = Math.floor((2 * minutes * 60) / stepSeconds) + 1;
        const geometry = new BufferGeometry();
        const positions = new Float32Array(numSteps * 3);
        geometry.setAttribute('position', new BufferAttribute(positions, 3));
        const material = new LineBasicMaterial({ color });
        super(geometry, material);

        // instance
        this.propagator = propagator;
        this.minutes = minutes;
        this.stepSeconds = stepSeconds;
        this.numSteps = numSteps;
        this.positions = positions;
        this.KM2S = 1 / 10000; // sim scale
    }

    updateLinePointsIdx(idx, p) {
        const i = idx * 3;
        this.positions[i]     = p[0] * this.KM2S;
        this.positions[i + 1] = p[1] * this.KM2S;
        this.positions[i + 2] = p[2] * this.KM2S;
    }

    generateTrajectory(sat_state) {
        const state = structuredClone(sat_state);
        const totalSeconds = this.minutes * 60;
        for (let t = 0; t < totalSeconds; t += this.stepSeconds) {
            this.propagator.step(state, -this.stepSeconds);
        }
        for (let idx = 0; idx < this.numSteps; idx++) {
            this.updateLinePointsIdx(idx, state);
            this.propagator.step(state, this.stepSeconds);
        }
        this.geometry.attributes.position.needsUpdate = true;
        this.geometry.computeBoundingSphere();
    }
}

class SatelliteSprite extends Sprite {
    constructor(color=0xFFFFFF, name="sat", path='static/assets/sat.svg') {
        const satMat = new SpriteMaterial({
            map: new TextureLoader().load('static/assets/sat.svg'),
            color: color,

        });
        super(satMat);
        this.scale.set(.1,.1,.1);
        this.renderOrder = 1;
        // params
        this.state = null;
        this.name = name;
        this.color = color;

        this.objectToolTip = true;
    }

    setState(state){this.state = state;}

    getStateDeepCopy(){return JSON.parse(JSON.stringify(this.state));}

    handleObjectClick()
    {
        console.log(this.name);
    }
    resetScale(){ this.scale.set(.1,.1,.1); }
}

class SatelliteBuilder {
    constructor(scene, propagator) {
        this.scene = scene;
        this.propagator = propagator;
        this.previewSat = null;
        this.previewOrbit = null;
        this.satellites = [];
        this.KM2S = 1/10000;
        
        this.defaultValues = {
            'sat-name': 'New Satellite',
            'sat-color': '#ff0000',
            'semi-major': '6778',
            'eccentricity': '0.0',
            'inclination': '0.0',
            'raan': '0',
            'arg-periapsis': '0',
            'true-anomaly': '0'
        };
        
        this.setupEventListeners();
        this.createPreviewSatellite();
    }

    createPreviewSatellite() {
        const elements = this.getOrbitalElements();
        const color = document.getElementById('sat-color').value;
        const name = document.getElementById('sat-name').value;

        this.previewSat = new SatelliteSprite(color, 'preview');
        this.previewSat.material.opacity = 0.6;
        this.previewSat.material.transparent = true;
        this.previewSat.setState(elementsToState(elements.a, elements.e, elements.i, elements.argPer, elements.raan, elements.nu));
        this.previewSat.position.set(
            this.previewSat.state[0] * this.KM2S,
            this.previewSat.state[1] * this.KM2S,
            this.previewSat.state[2] * this.KM2S
        );

        this.previewOrbit = new EciTrajectory(this.propagator, color);
        this.previewOrbit.material.opacity = 0.6;
        this.previewOrbit.material.transparent = true;
        this.previewOrbit.generateTrajectory(this.previewSat.getStateDeepCopy());

        this.scene.add(this.previewSat);
        this.scene.add(this.previewOrbit);
        
        this.previewSat.visible = false;
        this.previewOrbit.visible = false;
    }

    setupEventListeners() {
        document.getElementById('preview-btn').addEventListener('click', () => {
            this.togglePreview();
        });

        document.getElementById('create-btn').addEventListener('click', () => {
            this.createSatellite();
        });

        document.getElementById('reset-btn').addEventListener('click', () => {
            this.resetToDefaults();
        });

        const inputs = ['semi-major', 'eccentricity', 'inclination', 'raan', 'arg-periapsis', 'true-anomaly'];
        inputs.forEach(id => {
            document.getElementById(id).addEventListener('input', () => {
                this.updatePreviewPosition();
            });
        });

        document.getElementById('sat-color').addEventListener('input', () => {
            this.updatePreviewColor();
        });
    }

    togglePreview() {
        this.previewSat.visible = !this.previewSat.visible;
        this.previewOrbit.visible = !this.previewOrbit.visible;
        if(!this.previewSat.visible){this.previewSat.scale.set(0, 0, 0);}
        else{this.previewSat.resetScale();}
    }

    hidePreview() {
        this.previewSat.visible = false;
        this.previewOrbit.visible = false;
        if(!this.previewSat.visible){this.previewSat.scale.set(0, 0, 0);}
        else{this.previewSat.resetScale();}
    }

    updatePreviewPosition() {
        const elements = this.getOrbitalElements();
        const newState = elementsToState(elements.a, elements.e, elements.i, elements.argPer, elements.raan, elements.nu);
        this.previewSat.setState(newState);
        this.previewSat.position.set(
            this.previewSat.state[0] * this.KM2S,
            this.previewSat.state[1] * this.KM2S,
            this.previewSat.state[2] * this.KM2S
        );
        this.previewOrbit.generateTrajectory(this.previewSat.getStateDeepCopy());
    }

    updatePreviewColor() {
        const color = document.getElementById('sat-color').value;
        this.previewSat.material.color.setHex(color.replace('#', '0x'));
        this.previewOrbit.material.color.setHex(color.replace('#', '0x'));
    }

    resetToDefaults() {
        Object.keys(this.defaultValues).forEach(id => {
            const element = document.getElementById(id);
            if (element) {
                element.value = this.defaultValues[id];
            }
        });
        this.updatePreviewPosition();
        this.updatePreviewColor();

        console.log('Control panel reset to default values');
    }

    getOrbitalElements() {
        const a = parseFloat(document.getElementById('semi-major').value) || parseFloat(this.defaultValues['semi-major']);
        const e = parseFloat(document.getElementById('eccentricity').value) || parseFloat(this.defaultValues['eccentricity']);
        const i = (parseFloat(document.getElementById('inclination').value) || parseFloat(this.defaultValues['inclination'])) * Math.PI / 180;
        const raan = (parseFloat(document.getElementById('raan').value) || parseFloat(this.defaultValues['raan'])) * Math.PI / 180;
        const argPer = (parseFloat(document.getElementById('arg-periapsis').value) || parseFloat(this.defaultValues['arg-periapsis'])) * Math.PI / 180;
        const nu = (parseFloat(document.getElementById('true-anomaly').value) || parseFloat(this.defaultValues['true-anomaly'])) * Math.PI / 180;
        
        return { a, e, i, raan, argPer, nu };
    }

    createSatellite() {
        const elements = this.getOrbitalElements();
        const color = document.getElementById('sat-color').value;
        const name = document.getElementById('sat-name').value;
        const satellite = new SatelliteSprite(color, name);
        satellite.setState(elementsToState(elements.a, elements.e, elements.i, elements.argPer, elements.raan, elements.nu));
        satellite.position.set(
            satellite.state[0] * this.KM2S,
            satellite.state[1] * this.KM2S,
            satellite.state[2] * this.KM2S
        );
        const orbit = new EciTrajectory(this.propagator, color);
        orbit.generateTrajectory(satellite.getStateDeepCopy());

        this.scene.add(satellite);
        this.scene.add(orbit);

        this.satellites.push(satellite);

        satellite.originalElements = {
            a: elements.a,
            e: elements.e,
            i: elements.i,
            raan: elements.raan,
            argPer: elements.argPer,
            nu: elements.nu
        };

        console.log(`Created satellite: ${name}`);
    }
    
    getAllSatellites() {
        return this.satellites;
    }
    

    getAllSatelliteData() {
        return this.satellites.map(satellite => ({
            name: satellite.name,
            color: satellite.color,
            semiMajorAxis: satellite.originalElements.a,
            eccentricity: satellite.originalElements.e,
            inclination: satellite.originalElements.i * 180 / Math.PI,
            raan: satellite.originalElements.raan * 180 / Math.PI,
            argPeriapsis: satellite.originalElements.argPer * 180 / Math.PI,
            trueAnomaly: satellite.originalElements.nu * 180 / Math.PI
        }));
    }

}

class LineIndicator extends Line {
    constructor(color = 0x0000ff) {
        const geometry = new BufferGeometry();
        const positions = new Float32Array(6);
        geometry.setAttribute('position', new BufferAttribute(positions, 3));

        const material = new LineDashedMaterial({
            color: color,
            linewidth: 1,
            dashSize: .01,
            gapSize: 0.0075
        });
        super(geometry, material);
        this.computeLineDistances();
    }

    setEndpoints(x1, y1, z1, x2, y2, z2) {
        const pos = this.geometry.attributes.position.array;

        pos[0] = x1;
        pos[1] = y1;
        pos[2] = z1;

        pos[3] = x2;
        pos[4] = y2;
        pos[5] = z2;

        this.geometry.attributes.position.needsUpdate = true;
        this.computeLineDistances();
    }
}

class WedgeGeometry extends BoxGeometry {
    constructor( startRadius, endRadius, dPhi, dTheta ) {
        super( endRadius-startRadius, dPhi, dTheta,
                     1, 1+Math.round(dPhi*20), 1+Math.round(dTheta*40) );
        this.translate( (endRadius-startRadius)/2+startRadius, 0, 0 );
        var pos = this.getAttribute( 'position' ),
                v = new Vector3( );
        for( var i=0; i<pos.count; i++ ) {
            v.setFromSphericalCoords(
                        pos.getX(i),
                        pos.getY(i)-Math.PI/2,
                        -pos.getZ(i)-Math.PI/2 );
            pos.setXYZ( i, v.x, v.y, v.z );
        }
        this.computeVertexNormals( );
    }
}


class GroundStation extends Group {
    constructor(lat, lon, start_r, end_r, dPhi, dTheta, name="groundSation", color=0x1e90ff) {
        super();
        
        this.name = name;
        this.KM2S = 1/10000;
        this.earthRaduis = .6371;
        this.lat = lat;
        this.lon = lon;
        this.start_r = start_r;
        this.end_r = end_r;
        this.dPhi = dPhi * Math.PI / 180;
        this.dTheta = dTheta * Math.PI / 180;
        
        this.mesh = null;
        this.frame = null;
        this.baseMesh = null;
        this.offset = new Vector3(this.earthRaduis, 0, 0);

        this.color = color;

        this.setFromParams(this.color);

        this.worldPos = new Vector3(); // funny name since here world means the scene axis, not earth.
    }

    setFromParams() {

        const startRadius = this.start_r;
        const endRadius = this.end_r;
        const color = this.color
        const geo = new WedgeGeometry(startRadius, endRadius, this.dPhi, this.dTheta);
        const transparentMaterial = new MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.1,
            depthWrite: false,
            side: DoubleSide
        });
        this.mesh = new Mesh(geo, transparentMaterial);
        this.mesh.renderOrder = 0;

// const maxAngle = Math.max(this.dPhi, this.dTheta) * 180 / Math.PI;
// const thresholdAngle = Math.max(5, Math.min(25, maxAngle * 0.1));

        const edges = new EdgesGeometry(geo, 1);
        const lineMaterial = new LineBasicMaterial({ color: color });
        this.frame = new LineSegments(edges, lineMaterial);
        this.baseMesh = new Mesh(
            new TetrahedronGeometry(0.015, 2),
            new MeshPhongMaterial({
                color: color,
                flatShading: true,
                transparent: true,
                opacity: 0.75
            })
        );
        this.frame.position.copy(this.offset);
        this.mesh.position.copy(this.offset);
        this.baseMesh.position.copy(this.offset);
        this.add(this.frame);
        this.add(this.mesh);
        this.add(this.baseMesh);
        this.rotation.z = this.lat * Math.PI / 180;
        this.rotation.y = this.lon * Math.PI / 180;
        this.userData = {
            name: this.name,
            uuid: this.uuid,
            lat: this.lat,
            lon: this.lon,
            start_r: this.start_r,
            end_r: this.end_r,
            dPhi: this.dPhi,
            dTheta: this.dTheta
        };

        this.baseMesh.name = this.name;
        this.baseMesh.objectToolTip = true;

        this.baseMesh.handleObjectClick = function() {
            console.log(this.name);
        };
    }

    setWorldPos(){
        this.baseMesh.getWorldPosition(this.worldPos);
    }
}


class RadarBuilder {
    constructor(scene, intersections = null) {
        this.scene = scene;
        this.intersections = intersections;
        this.previewRadar = null;
        this.radars = [];
        this.KM2S = 1/10000;
        this.defaultValues = {
            'radar-name': 'New Radar',
            'radar-color': '#1e90ff',
            'radar-lat': '0',
            'radar-lon': '0',
            'radar-start-r': '0.75',
            'radar-end-r': '4.5',
            'radar-dphi': '7',
            'radar-dtheta': '12'
        };
        
        this.setupEventListeners();
        this.createPreviewRadar();
        this.previewRadar.visible = false;
    }
    setIntersections(intersections) {
        this.intersections = intersections;
        if (this.previewRadar && this.previewRadar.baseMesh) {
            this.intersections.push(this.previewRadar.baseMesh);
        }
    }

    updatePreview() {
        const wasVisible = this.previewRadar ? this.previewRadar.visible : false;
        if (this.previewRadar) {
            this.scene.remove(this.previewRadar);
            if (this.intersections && this.previewRadar.baseMesh) {
                const oldIndex = this.intersections.indexOf(this.previewRadar.baseMesh);
                if (oldIndex !== -1) {
                    this.intersections.splice(oldIndex, 1);
                }
            }
        }
        this.createPreviewRadar();
        if (this.intersections && this.previewRadar.baseMesh) {
            this.intersections.push(this.previewRadar.baseMesh);
        }
        this.previewRadar.visible = wasVisible;
    }
    createPreviewRadar() {
        const params = this.getRadarParameters();
        this.previewRadar = new GroundStation(
            params.lat, params.lon, params.start_r, params.end_r, 
            params.dPhi, params.dTheta, 'preview', params.color
        );
        this.previewRadar.mesh.material.opacity = 0.3;
        this.previewRadar.frame.material.opacity = 0.6;
        this.previewRadar.baseMesh.material.opacity = 0.3;
        this.scene.add(this.previewRadar);
    }

    setupEventListeners() {
        document.getElementById('radar-preview-btn').addEventListener('click', () => {
            this.togglePreview();
        });

        document.getElementById('radar-create-btn').addEventListener('click', () => {
            this.createRadar();
        });

        document.getElementById('radar-reset-btn').addEventListener('click', () => {
            this.resetToDefaults();
        });

        const inputs = ['radar-lat', 'radar-lon', 'radar-start-r', 'radar-end-r', 'radar-dphi', 'radar-dtheta'];
        inputs.forEach(id => {
            document.getElementById(id).addEventListener('input', () => {
                this.updatePreview();
            });
        });

        document.getElementById('radar-color').addEventListener('input', () => {
            this.updatePreview();
        });
    }

    togglePreview() {
        this.previewRadar.visible = !this.previewRadar.visible;
    }

    hidePreview() {
        this.previewRadar.visible = false;
    }


    resetToDefaults() {
        Object.keys(this.defaultValues).forEach(id => {
            const element = document.getElementById(id);
            if (element) {
                element.value = this.defaultValues[id];
            }
        });
        this.updatePreview();
        console.log('Radar panel reset to default values');
    }

    getRadarParameters() {
        const lat = parseFloat(document.getElementById('radar-lat').value) || parseFloat(this.defaultValues['radar-lat']);
        const lon = parseFloat(document.getElementById('radar-lon').value) || parseFloat(this.defaultValues['radar-lon']);
        const start_r = parseFloat(document.getElementById('radar-start-r').value) || parseFloat(this.defaultValues['radar-start-r']);
        const end_r = parseFloat(document.getElementById('radar-end-r').value) || parseFloat(this.defaultValues['radar-end-r']);
        const dPhi = parseFloat(document.getElementById('radar-dphi').value) || parseFloat(this.defaultValues['radar-dphi']);
        const dTheta = parseFloat(document.getElementById('radar-dtheta').value) || parseFloat(this.defaultValues['radar-dtheta']);
        const colorHex = document.getElementById('radar-color').value;
        const color = parseInt(colorHex.replace('#', '0x'), 16);
        
        return { lat, lon, start_r, end_r, dPhi, dTheta, color };
    }

    createRadar() {
        const params = this.getRadarParameters();
        const name = document.getElementById('radar-name').value;

        const radar = new GroundStation(
            params.lat, params.lon, params.start_r, params.end_r, 
            params.dPhi, params.dTheta, name, params.color
        );

        this.scene.add(radar);
        this.radars.push(radar);

        if (this.intersections && radar.baseMesh) {
            this.intersections.push(radar.baseMesh);
        }

        radar.originalParams = {
            name: name,
            lat: params.lat,
            lon: params.lon,
            start_r: params.start_r,
            end_r: params.end_r,
            dPhi: params.dPhi,
            dTheta: params.dTheta,
            color: document.getElementById('radar-color').value
        };

        console.log(`Created radar: ${name}`);
    }
    
    getAllRadars() {
        return this.radars;
    }

    getAllRadarData() {
        return this.radars.map(radar => ({
            name: radar.originalParams.name,
            lat: radar.originalParams.lat,
            lon: radar.originalParams.lon,
            start_r: radar.originalParams.start_r,
            end_r: radar.originalParams.end_r,
            dPhi: radar.originalParams.dPhi,
            dTheta: radar.originalParams.dTheta,
            color: radar.originalParams.color
        }));
    }
}

class CrossHairSprite extends Sprite {
    constructor(color=0xFFFFFF, name="crosshair", path='static/assets/crosshair.svg') {
        const satMat = new SpriteMaterial({
            map: new TextureLoader().load(path),
            color: color
        });
        super(satMat);
        this.scale.set(.15,.15, .15);
        // params
        this.name = name;
        this.color = color;
        this.objectToolTip = true;
        this.renderOrder = 2;
    }
    handleObjectClick()
    {
        // console.log(this.name);
        return
    }

}

function hexToRgb01(hex) {
    // Ensure it's a number (like 0x00ff04)
    const r = (hex >> 16) & 0xff;
    const g = (hex >> 8) & 0xff;
    const b = hex & 0xff;

    return {
        r: r / 255,
        g: g / 255,
        b: b / 255
    };
}

class DynPoints extends Points {
    constructor( N, color = 0x880808, sprite = null )
    {
        const colors = [];
        const vertices = [];
        const visibility = [];
        const sizes = [];
        const rgb = hexToRgb01(color);
        for (let i = 0; i < N; i++)
        {
            vertices.push( 0, 0, 0);
            colors.push(rgb.r, rgb.g, rgb.b);
            visibility.push(0);
            sizes.push(.015);
        }
        const geometry = new BufferGeometry();
        geometry.setAttribute( 'position', new Float32BufferAttribute( vertices, 3 ) );
        geometry.setAttribute( 'color', new Float32BufferAttribute( colors, 3 ) );
        const material = new PointsMaterial({
            map : sprite ,
            size: 1,
            // blending: AdditiveBlending,
            vertexColors: true,
            // transparent: true,
            // depthWrite: false,
            onBeforeCompile: function(shader){
                shader.vertexShader = `
                attribute float sizes;
                attribute float visibility;
                varying float vVisible;
                ${shader.vertexShader}`
                .replace(
                    `gl_PointSize = size;`,
                    `gl_PointSize = size * sizes;
                    vVisible = visibility;
                    `
                );
                shader.fragmentShader = `
                varying float vVisible;
                ${shader.fragmentShader}`
                .replace(
                    `#include <clipping_planes_fragment>`,
                    `
                    if (vVisible < 0.5) discard;
                    #include <clipping_planes_fragment>`
                )
            }
        });

        super( geometry, material );
        this.geo = geometry;
        this.positionAttribute = this.geo.getAttribute( 'position' );
        // this.positionAttribute.needsUpdate = true;
        this.posArray = this.positionAttribute.array;
        this.colorAttribute = this.geo.getAttribute( 'color' );
        this.colorArray = this.colorAttribute.array;
        this.colorAttribute.needsUpdate = true;

        // from shader code
        this.geo.setAttribute("visibility", new Float32BufferAttribute(visibility, 1));
        this.visibilityAttribute = this.geo.getAttribute( 'visibility' );
        this.visibilityArray = this.visibilityAttribute.array;
        // this.visibilityAttribute.needsUpdate = true;
        this.geo.setAttribute("sizes", new Float32BufferAttribute(sizes, 1));
        this.sizeAttribute = this.geo.getAttribute( 'sizes' );
        // this.sizeAttribute.needsUpdate = true;
        this.sizeArray = this.sizeAttribute.array;
        //
        this.renderOrder = 1;
        this.N = N;
        this._idx = 0;
    }
    posUpdate() {this.positionAttribute.needsUpdate = true;}
    colorUpdate() {this.colorAttribute.needsUpdate = true;}
    sizeUpdate() {this.sizeAttribute.needsUpdate = true;}
    visUpdate() {this.visibilityAttribute.needsUpdate = true;}
    setPosIdx( idx, px, py, pz )
    {
        this.posArray[idx*3  ] = px;
        this.posArray[idx*3+1] = py;
        this.posArray[idx*3+2] = pz;
        this.visibilityArray[idx] = 1;
        this.positionAttribute.needsUpdate = true;
        this.visibilityAttribute.needsUpdate = true;
    }
    cyclePos( px, py, pz )
    {
        this._idx = this._idx%this.N;
        this._idx += 1;
        this.setPosIdx(this._idx,px, py, pz)
    }
    setColorIdx( idx, cx, cy, cz )
    {
        this.colorArray[idx*3  ] = cx;
        this.colorArray[idx*3+1] = cy;
        this.colorArray[idx*3+2] = cz;
        this.colorAttribute.needsUpdate = true;
    }
    setSizeIdx( idx, s )
    {
        this.sizeArray[idx ] = s;
        this.sizeAttribute.needsUpdate = true;
    }
    setVisibilityAll( val )
    {
        // val should be 1 or 0
        for(let i=0; i<this.N; i+=1)
        {
            this.visibilityArray[i] = val;
        }
    }
}