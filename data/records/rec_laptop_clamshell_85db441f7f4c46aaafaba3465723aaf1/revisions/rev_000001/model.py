from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workstation_laptop")

    chassis_dark = model.material("chassis_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    chassis_mid = model.material("chassis_mid", rgba=(0.28, 0.29, 0.31, 1.0))
    deck_dark = model.material("deck_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    key_dark = model.material("key_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    tray_dark = model.material("tray_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.17, 0.21, 0.92))
    touchpad_mat = model.material("touchpad", rgba=(0.18, 0.19, 0.20, 1.0))
    camera_mat = model.material("camera", rgba=(0.03, 0.03, 0.04, 1.0))

    base = model.part("base")
    # Perimeter shell and deck framing for a heavy mobile-workstation base.
    base.visual(
        Box((0.280, 0.010, 0.030)),
        origin=Origin(xyz=(0.000, -0.185, 0.015)),
        material=chassis_dark,
        name="left_wall",
    )
    base.visual(
        Box((0.280, 0.010, 0.030)),
        origin=Origin(xyz=(0.000, 0.185, 0.015)),
        material=chassis_dark,
        name="right_wall",
    )
    base.visual(
        Box((0.010, 0.370, 0.024)),
        origin=Origin(xyz=(-0.135, 0.000, 0.012)),
        material=chassis_dark,
        name="front_wall",
    )
    base.visual(
        Box((0.054, 0.370, 0.006)),
        origin=Origin(xyz=(-0.113, 0.000, 0.027)),
        material=chassis_mid,
        name="front_fascia",
    )
    base.visual(
        Box((0.014, 0.370, 0.030)),
        origin=Origin(xyz=(0.133, 0.000, 0.015)),
        material=chassis_dark,
        name="rear_wall",
    )
    base.visual(
        Box((0.090, 0.370, 0.004)),
        origin=Origin(xyz=(-0.092, 0.000, 0.030)),
        material=chassis_mid,
        name="palmrest_deck",
    )
    base.visual(
        Box((0.056, 0.370, 0.004)),
        origin=Origin(xyz=(0.107, 0.000, 0.030)),
        material=chassis_mid,
        name="rear_deck",
    )
    base.visual(
        Box((0.126, 0.056, 0.004)),
        origin=Origin(xyz=(0.014, -0.157, 0.030)),
        material=chassis_mid,
        name="left_keyboard_rail",
    )
    base.visual(
        Box((0.126, 0.056, 0.004)),
        origin=Origin(xyz=(0.014, 0.157, 0.030)),
        material=chassis_mid,
        name="right_keyboard_rail",
    )
    base.visual(
        Box((0.126, 0.012, 0.002)),
        origin=Origin(xyz=(0.012, -0.137, 0.02465)),
        material=chassis_dark,
        name="left_tray_ledge",
    )
    base.visual(
        Box((0.126, 0.012, 0.002)),
        origin=Origin(xyz=(0.012, 0.137, 0.02465)),
        material=chassis_dark,
        name="right_tray_ledge",
    )
    base.visual(
        Box((0.126, 0.010, 0.006)),
        origin=Origin(xyz=(0.012, -0.146, 0.027)),
        material=chassis_dark,
        name="left_tray_bracket",
    )
    base.visual(
        Box((0.126, 0.010, 0.006)),
        origin=Origin(xyz=(0.012, 0.146, 0.027)),
        material=chassis_dark,
        name="right_tray_bracket",
    )
    base.visual(
        Box((0.022, 0.028, 0.002)),
        origin=Origin(xyz=(0.124, -0.142, 0.032)),
        material=chassis_mid,
        name="left_hinge_block",
    )
    base.visual(
        Box((0.022, 0.028, 0.002)),
        origin=Origin(xyz=(0.124, 0.142, 0.032)),
        material=chassis_mid,
        name="right_hinge_block",
    )
    base.visual(
        Box((0.100, 0.066, 0.0008)),
        origin=Origin(xyz=(-0.095, 0.000, 0.0324)),
        material=touchpad_mat,
        name="touchpad",
    )
    # Underside closure plates leave a dedicated opening for the service door.
    base.visual(
        Box((0.252, 0.240, 0.003)),
        origin=Origin(xyz=(0.000, -0.060, 0.0015)),
        material=chassis_dark,
        name="bottom_main",
    )
    base.visual(
        Box((0.140, 0.110, 0.003)),
        origin=Origin(xyz=(-0.070, 0.115, 0.0015)),
        material=chassis_dark,
        name="bottom_front_right",
    )
    base.visual(
        Box((0.035, 0.110, 0.003)),
        origin=Origin(xyz=(0.1225, 0.115, 0.0015)),
        material=chassis_dark,
        name="bottom_rear_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.280, 0.380, 0.032)),
        mass=2.8,
        origin=Origin(xyz=(0.000, 0.000, 0.016)),
    )

    keyboard_tray = model.part("keyboard_tray")
    keyboard_tray.visual(
        Box((0.120, 0.286, 0.0015)),
        origin=Origin(xyz=(0.000, 0.000, 0.00075)),
        material=tray_dark,
        name="tray_plate",
    )
    keyboard_tray.inertial = Inertial.from_geometry(
        Box((0.120, 0.286, 0.0015)),
        mass=0.12,
        origin=Origin(xyz=(0.000, 0.000, 0.00075)),
    )
    model.articulation(
        "base_to_keyboard_tray",
        ArticulationType.FIXED,
        parent=base,
        child=keyboard_tray,
        origin=Origin(xyz=(0.012, 0.000, 0.02565)),
    )

    key_size = (0.019, 0.020, 0.003)
    key_rows = (-0.036, -0.012, 0.012, 0.036)
    key_cols = tuple(-0.1125 + 0.025 * col for col in range(10))
    articulated_keys = {(1, 4), (1, 5), (2, 4), (2, 5)}

    for row_index, x_pos in enumerate(key_rows):
        for col_index, y_pos in enumerate(key_cols):
            if (row_index, col_index) in articulated_keys:
                continue
            keyboard_tray.visual(
                Box(key_size),
                origin=Origin(xyz=(x_pos, y_pos, 0.003)),
                material=key_dark,
                name=f"fixed_key_r{row_index}_c{col_index}",
            )

    for row_index, col_index in sorted(articulated_keys):
        x_pos = key_rows[row_index]
        y_pos = key_cols[col_index]
        key_name = f"key_r{row_index}_c{col_index}"
        key_part = model.part(key_name)
        key_part.visual(
            Box(key_size),
            origin=Origin(xyz=(0.000, 0.000, 0.0015)),
            material=key_dark,
            name="keycap",
        )
        key_part.inertial = Inertial.from_geometry(
            Box(key_size),
            mass=0.004,
            origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        )
        model.articulation(
            f"tray_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=keyboard_tray,
            child=key_part,
            origin=Origin(xyz=(x_pos, y_pos, 0.0015)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=0.05,
                lower=0.0,
                upper=0.0010,
            ),
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.270, 0.372, 0.007)),
        origin=Origin(xyz=(-0.135, 0.000, 0.0035)),
        material=chassis_dark,
        name="lid_back_shell",
    )
    lid.visual(
        Box((0.050, 0.372, 0.003)),
        origin=Origin(xyz=(-0.025, 0.000, 0.0085)),
        material=deck_dark,
        name="bottom_bezel",
    )
    lid.visual(
        Box((0.020, 0.372, 0.003)),
        origin=Origin(xyz=(-0.260, 0.000, 0.0085)),
        material=deck_dark,
        name="top_bezel",
    )
    lid.visual(
        Box((0.200, 0.026, 0.003)),
        origin=Origin(xyz=(-0.150, -0.173, 0.0085)),
        material=deck_dark,
        name="left_bezel",
    )
    lid.visual(
        Box((0.200, 0.026, 0.003)),
        origin=Origin(xyz=(-0.150, 0.173, 0.0085)),
        material=deck_dark,
        name="right_bezel",
    )
    lid.visual(
        Box((0.200, 0.320, 0.0015)),
        origin=Origin(xyz=(-0.150, 0.000, 0.00775)),
        material=glass,
        name="display_panel",
    )
    lid.visual(
        Box((0.006, 0.006, 0.001)),
        origin=Origin(xyz=(-0.258, 0.000, 0.0095)),
        material=camera_mat,
        name="webcam",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.270, 0.372, 0.010)),
        mass=0.95,
        origin=Origin(xyz=(-0.135, 0.000, 0.005)),
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.133, 0.000, 0.033)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=0.0,
            upper=2.05,
        ),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.100, 0.102, 0.003)),
        origin=Origin(xyz=(0.000, -0.051, -0.0015)),
        material=deck_dark,
        name="service_panel",
    )
    service_door.visual(
        Box((0.020, 0.018, 0.0015)),
        origin=Origin(xyz=(-0.030, -0.084, -0.00375)),
        material=chassis_mid,
        name="service_pull_lip",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.100, 0.102, 0.003)),
        mass=0.08,
        origin=Origin(xyz=(0.000, -0.051, -0.0015)),
    )
    model.articulation(
        "base_to_service_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=service_door,
        origin=Origin(xyz=(0.052, 0.162, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.2,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    keyboard_tray = object_model.get_part("keyboard_tray")
    lid = object_model.get_part("lid")
    service_door = object_model.get_part("service_door")
    sample_key = object_model.get_part("key_r1_c4")

    lid_hinge = object_model.get_articulation("base_to_lid")
    service_hinge = object_model.get_articulation("base_to_service_door")
    sample_key_joint = object_model.get_articulation("tray_to_key_r1_c4")

    ctx.check("base exists", base is not None)
    ctx.check("keyboard tray exists", keyboard_tray is not None)
    ctx.check("lid exists", lid is not None)
    ctx.check("service door exists", service_door is not None)
    ctx.check("sample key exists", sample_key is not None)

    with ctx.pose({lid_hinge: 0.0, service_hinge: 0.0, sample_key_joint: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid sits just above the base",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.22,
            name="closed lid covers the base footprint",
        )
        ctx.expect_gap(
            base,
            service_door,
            axis="z",
            max_gap=0.0006,
            max_penetration=0.0,
            name="service door closes flush to the underside",
        )
        ctx.expect_overlap(
            service_door,
            base,
            axes="xy",
            min_overlap=0.09,
            name="service door remains mounted under the base",
        )
        ctx.expect_gap(
            sample_key,
            keyboard_tray,
            axis="z",
            negative_elem="tray_plate",
            max_gap=0.0002,
            max_penetration=0.0,
            name="resting key is supported by the switch tray",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_back_shell")
    with ctx.pose({lid_hinge: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_back_shell")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(service_door, elem="service_panel")
    with ctx.pose({service_hinge: 1.10}):
        open_door_aabb = ctx.part_element_world_aabb(service_door, elem="service_panel")
    ctx.check(
        "service door swings downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.035,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_key_aabb = ctx.part_element_world_aabb(sample_key, elem="keycap")
    key_upper = sample_key_joint.motion_limits.upper if sample_key_joint.motion_limits is not None else None
    with ctx.pose({sample_key_joint: key_upper or 0.0010}):
        pressed_key_aabb = ctx.part_element_world_aabb(sample_key, elem="keycap")
    ctx.check(
        "key plunges downward",
        rest_key_aabb is not None
        and pressed_key_aabb is not None
        and pressed_key_aabb[1][2] < rest_key_aabb[1][2] - 0.0008,
        details=f"rest={rest_key_aabb}, pressed={pressed_key_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
