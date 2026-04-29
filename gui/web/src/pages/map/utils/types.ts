export class MowingAreaEdit {
    id?: string;
    name: string;
    mowing_order: number;
    orig_mowing_order: number;
    feature_type: string;
    orig_feature_type: string;
    index: number;
    /**
     * Per-area narrow-area handling strategy — mirrors
     * `MapArea.narrow_area_strategy` in `mowgli_interfaces/msg/MapArea.msg`.
     * 0 = SKIP (default), 1 = OUTLINE_ONLY, 2 = SPECIAL_PATTERN.
     * Optional to ease migration of legacy area data; the default falls back
     * to 0 (Skip) at write time to match the .msg field's uint8 zero-default.
     */
    narrow_area_strategy?: number;

    constructor() {
        this.name = '';
        this.mowing_order = 9999;
        this.orig_mowing_order = 9999;
        this.feature_type = 'workarea';
        this.orig_feature_type = 'workarea';
        this.index = -1;
        this.narrow_area_strategy = 0;
    }
}

export interface AreaListItem {
    id: string;
    name: string;
    ftype: string;
    areaLabel: string;
    mowingOrder?: number;
}
