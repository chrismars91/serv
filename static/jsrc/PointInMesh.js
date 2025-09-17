class PointInMeshChecker {
    constructor() {
        this._raycaster = new Raycaster();
        this._dirWorld = new Vector3(1, 0.123456, 0.54321).normalize();
        this._rayOrigin = new Vector3();
        this._pointWorld = new Vector3();
        this.inMesh = false;
        this.notTracking = false;
    }

    setPointWorld(x, y, z) {
        this._pointWorld.set(x, y, z);
    }
    isPointInsideMesh(mesh) {
        if (this.notTracking){return;}
        this._rayOrigin.copy(this._pointWorld).addScaledVector(this._dirWorld, 1e-6);
        this._raycaster.set(this._rayOrigin, this._dirWorld);
        const intersects = this._raycaster.intersectObject(mesh, true);
        let count = 0;
        for (let i = 0; i < intersects.length; i++) {
            if (intersects[i].distance > 1e-7) count++;
        }
        this.inMesh = (count % 2) === 1;
    }
}