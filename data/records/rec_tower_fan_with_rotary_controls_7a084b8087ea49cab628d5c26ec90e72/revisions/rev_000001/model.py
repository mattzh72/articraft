from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    mesh_from_geometry,
)


def _build_grille_mesh():
    return mesh_from_geometry(
        VentGrilleGeometry(
            panel_size=(0.114, 0.398),
            frame=0.010,
            face_thickness=0.0035,
            duct_depth=0.014,
            duct_wall=0.0025,
            slat_pitch=0.014,
            slat_width=0.006,
            slat_angle_deg=18.0,
            corner_radius=0.014,
        ),
        "tower_fan_front_grille",
    )


def _build_impeller_mesh():
    return mesh_from_geometry(
        BlowerWheelGeometry(
            outer_radius=0.028,
            inner_radius=0.013,
            width=0.388,
            blade_count=28,
            blade_thickness=0.0018,
            blade_sweep_deg=34.0,
            backplate=True,
            shroud=True,
        ),
        "tower_fan_impeller",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dorm_tower_fan")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    housing = model.material("housing", rgba=(0.92, 0.93, 0.90, 1.0))
    trim = model.material("trim", rgba=(0.58, 0.60, 0.62, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    impeller_dark = model.material("impeller_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    indicator = model.material("indicator", rgba=(0.76, 0.77, 0.78, 1.0))

    grille_mesh = _build_grille_mesh()
    impeller_mesh = _build_impeller_mesh()

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.165, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=base_dark,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.148, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim,
        name="base_trim_ring",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=trim,
        name="pivot_pedestal",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.030, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=trim,
        name="pivot_collar",
    )
    column.visual(
        Box((0.132, 0.090, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=housing,
        name="lower_plinth",
    )
    column.visual(
        Box((0.006, 0.090, 0.436)),
        origin=Origin(xyz=(-0.063, 0.0, 0.288)),
        material=housing,
        name="side_wall_0",
    )
    column.visual(
        Box((0.006, 0.090, 0.436)),
        origin=Origin(xyz=(0.063, 0.0, 0.288)),
        material=housing,
        name="side_wall_1",
    )
    column.visual(
        Box((0.120, 0.006, 0.436)),
        origin=Origin(xyz=(0.0, -0.042, 0.288)),
        material=housing,
        name="rear_wall",
    )
    column.visual(
        Box((0.116, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, 0.042, 0.086)),
        material=housing,
        name="front_rail_lower",
    )
    column.visual(
        Box((0.116, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.042, 0.492)),
        material=housing,
        name="front_rail_upper",
    )
    column.visual(
        Box((0.014, 0.006, 0.388)),
        origin=Origin(xyz=(-0.055, 0.042, 0.290)),
        material=housing,
        name="front_stile_0",
    )
    column.visual(
        Box((0.014, 0.006, 0.388)),
        origin=Origin(xyz=(0.055, 0.042, 0.290)),
        material=housing,
        name="front_stile_1",
    )
    column.visual(
        Box((0.138, 0.094, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.516)),
        material=housing,
        name="top_cap",
    )
    column.visual(
        Box((0.086, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.002, 0.525)),
        material=trim,
        name="control_pad",
    )
    column.visual(
        grille_mesh,
        origin=Origin(xyz=(0.0, 0.040, 0.290), rpy=(-1.5707963267948966, 0.0, 0.0)),
        material=grille_dark,
        name="front_grille",
    )

    impeller = model.part("impeller")
    impeller.visual(
        impeller_mesh,
        material=impeller_dark,
        name="impeller_wheel",
    )
    impeller.visual(
        Cylinder(radius=0.015, length=0.388),
        material=trim,
        name="impeller_hub",
    )
    impeller.visual(
        Cylinder(radius=0.0045, length=0.436),
        material=trim,
        name="impeller_shaft",
    )

    mode_knob = model.part("mode_knob")
    mode_knob.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=knob_dark,
        name="mode_stem",
    )
    mode_knob.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=knob_dark,
        name="mode_body",
    )
    mode_knob.visual(
        Box((0.004, 0.016, 0.003)),
        origin=Origin(xyz=(0.0, 0.014, 0.0245)),
        material=indicator,
        name="mode_indicator",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=knob_dark,
        name="timer_stem",
    )
    timer_knob.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=knob_dark,
        name="timer_body",
    )
    timer_knob.visual(
        Box((0.003, 0.010, 0.0025)),
        origin=Origin(xyz=(0.0, 0.009, 0.0185)),
        material=indicator,
        name="timer_indicator",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "column_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=impeller,
        origin=Origin(xyz=(0.0, -0.006, 0.286)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=40.0,
        ),
    )
    model.articulation(
        "column_to_mode_knob",
        ArticulationType.REVOLUTE,
        parent=column,
        child=mode_knob,
        origin=Origin(xyz=(-0.020, 0.004, 0.524)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=4.0,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "column_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=column,
        child=timer_knob,
        origin=Origin(xyz=(0.026, 0.004, 0.524)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=3.0,
            lower=0.0,
            upper=5.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    impeller = object_model.get_part("impeller")
    mode_knob = object_model.get_part("mode_knob")
    timer_knob = object_model.get_part("timer_knob")

    oscillation = object_model.get_articulation("base_to_column")
    impeller_spin = object_model.get_articulation("column_to_impeller")
    mode_joint = object_model.get_articulation("column_to_mode_knob")
    timer_joint = object_model.get_articulation("column_to_timer_knob")

    with ctx.pose({oscillation: 0.0}):
        ctx.expect_gap(
            column,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="pivot_collar",
            negative_elem="pivot_pedestal",
            name="column collar seats on base pedestal",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="xy",
            min_overlap=0.06,
            name="column stands within the base footprint",
        )
        ctx.expect_within(
            impeller,
            column,
            axes="xy",
            margin=0.012,
            elem_a="impeller_wheel",
            name="impeller stays inside the tower housing footprint",
        )
        ctx.expect_gap(
            mode_knob,
            column,
            axis="z",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="mode_body",
            negative_elem="top_cap",
            name="mode knob body stands proud of the top cap",
        )
        ctx.expect_gap(
            timer_knob,
            column,
            axis="z",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="timer_body",
            negative_elem="top_cap",
            name="timer knob body stands proud of the top cap",
        )
        ctx.expect_gap(
            timer_knob,
            mode_knob,
            axis="x",
            min_gap=0.006,
            name="top knobs remain visually distinct",
        )

    rest_timer_pos = ctx.part_world_position(timer_knob)
    with ctx.pose({oscillation: 0.75}):
        swung_timer_pos = ctx.part_world_position(timer_knob)

    ctx.check(
        "column oscillation sweeps the top controls sideways",
        rest_timer_pos is not None
        and swung_timer_pos is not None
        and (
            ((swung_timer_pos[0] - rest_timer_pos[0]) ** 2 + (swung_timer_pos[1] - rest_timer_pos[1]) ** 2)
            ** 0.5
        )
        > 0.015,
        details=f"rest={rest_timer_pos}, swung={swung_timer_pos}",
    )
    ctx.check(
        "impeller uses continuous spin articulation",
        impeller_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={impeller_spin.articulation_type}",
    )
    ctx.check(
        "top knobs rotate independently",
        mode_joint.child != timer_joint.child
        and mode_joint.parent == column.name
        and timer_joint.parent == column.name,
        details=(
            f"mode_joint=({mode_joint.parent}->{mode_joint.child}), "
            f"timer_joint=({timer_joint.parent}->{timer_joint.child})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
