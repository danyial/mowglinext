import {Form, Input, InputNumber, Modal, Select} from "antd";
import {MowingAreaEdit} from "../utils/types.ts";

interface EditAreaModalProps {
    open: boolean;
    area: MowingAreaEdit;
    onChange: (area: MowingAreaEdit) => void;
    onSave: () => void;
    onCancel: () => void;
}

const AREA_TYPE_OPTIONS = [
    {value: 'workarea', label: 'Mowing Area'},
    {value: 'navigation', label: 'Navigation Area'},
    {value: 'obstacle', label: 'Obstacle'},
];

// UI-SPEC §"narrow_area_strategy Dropdown" — labels are user-facing copy.
// Values match MapArea.narrow_area_strategy uint8 constants
// (0=SKIP, 1=OUTLINE_ONLY, 2=SPECIAL_PATTERN) per SPEC R-13 / Plan 01-01.
const NARROW_AREA_OPTIONS = [
    {value: 0, label: 'Skip'},
    {value: 1, label: 'Outline only'},
    {value: 2, label: 'Special pattern'},
];

export const EditAreaModal = ({open, area, onChange, onSave, onCancel}: EditAreaModalProps) => (
    <Modal
        open={open}
        title={area.name ? `Edit "${area.name}"` : "Edit area"}
        okText="Save"
        cancelText="Cancel"
        onOk={onSave}
        onCancel={onCancel}
        destroyOnClose
    >
        <Form layout="vertical" style={{marginTop: 16}}>
            <Form.Item label="Area type">
                <Select
                    value={area.feature_type}
                    onChange={(v) => onChange({...area, feature_type: v})}
                    options={AREA_TYPE_OPTIONS}
                />
            </Form.Item>
            {area.feature_type === 'workarea' && (
                <Form.Item label="Area name">
                    <Input
                        key="areaname"
                        placeholder="e.g. Front lawn"
                        value={area.name}
                        onChange={(e) => onChange({...area, name: e.target.value})}
                        autoFocus
                    />
                </Form.Item>
            )}
            {area.feature_type === 'workarea' && (
                <Form.Item label="Mowing order">
                    <InputNumber
                        key="mowingorder"
                        min={1}
                        value={area.mowing_order}
                        onChange={(v) => onChange({...area, mowing_order: v ?? 9999})}
                        style={{width: '100%'}}
                    />
                </Form.Item>
            )}
            {area.feature_type === 'workarea' && (
                <Form.Item
                    label="Narrow area strategy"
                    extra={
                        "Applies to strips shorter than 2× tool width. " +
                        "'Outline only' adds an extra pass along the center. " +
                        "'Special pattern' uses the area's long axis."
                    }
                >
                    <Select
                        key="narrowareastrategy"
                        value={area.narrow_area_strategy ?? 0}
                        onChange={(v) => onChange({...area, narrow_area_strategy: v})}
                        options={NARROW_AREA_OPTIONS}
                    />
                </Form.Item>
            )}
        </Form>
    </Modal>
);
