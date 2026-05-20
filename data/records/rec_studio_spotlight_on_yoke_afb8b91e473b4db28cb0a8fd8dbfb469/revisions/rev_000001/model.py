from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _save_mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _stand_mesh() -> MeshGeometry:
    leg_geometries: list[MeshGeometry] = [
        CylinderGeometry(radius=0.072, height=0.08, radial_segments=36).translate(0.0, 0.0, 0.105),
        CylinderGeometry(radius=0.040, height=0.58, radial_segments=36).translate(0.0, 0.0, 0.40),
        LatheGeometry.from_shell_profiles(
            [(0.058, 0.0), (0.058, 0.95)],
            [(0.036, 0.0), (0.036, 0.95)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ).translate(0.0, 0.0, 0.65),
        TorusGeometry(radius=0.058, tube=0.007, radial_segments=14, tubular_segments=56).translate(
            0.0, 0.0, 1.60
        ),
        CylinderGeometry(radius=0.014, height=0.17, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.112, 1.54),
        CylinderGeometry(radius=0.034, height=0.030, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.190, 1.54),
    ]

    for angle in (math.radians(90.0), math.radians(210.0), math.radians(330.0)):
        c, s = math.cos(angle), math.sin(angle)
        leg_geometries.append(
            tube_from_spline_points(
                [
                    (0.020 * c, 0.020 * s, 0.115),
                    (0.30 * c, 0.30 * s, 0.070),
                    (0.78 * c, 0.78 * s, 0.040),
                ],
                radius=0.024,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            )
        )
        leg_geometries.append(
            CylinderGeometry(radius=0.070, height=0.025, radial_segments=28).translate(
                0.82 * c, 0.82 * s, 0.020
            )
        )

    return _merge_geometries(leg_geometries)


def _center_column_mesh() -> MeshGeometry:
    return _merge_geometries(
        [
            CylinderGeometry(radius=0.028, height=1.45, radial_segments=40).translate(0.0, 0.0, -0.025),
            CylinderGeometry(radius=0.070, height=0.080, radial_segments=40).translate(0.0, 0.0, 0.740),
            TorusGeometry(radius=0.070, tube=0.006, radial_segments=12, tubular_segments=44).translate(
                0.0, 0.0, 0.700
            ),
        ]
    )


def _can_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.160, -0.360),
            (0.200, -0.300),
            (0.218, -0.140),
            (0.218, 0.330),
            (0.246, 0.430),
            (0.232, 0.520),
        ],
        [
            (0.132, -0.345),
            (0.176, -0.285),
            (0.190, -0.135),
            (0.190, 0.320),
            (0.205, 0.425),
            (0.198, 0.505),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    shell.merge(
        TorusGeometry(radius=0.226, tube=0.014, radial_segments=16, tubular_segments=72).translate(
            0.0, 0.0, 0.505
        )
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theatre_spotlight_stand")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    black_metal = model.material("black_metal", rgba=(0.055, 0.058, 0.064, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.45, 0.46, 0.47, 1.0))
    rubber = model.material("rubber", rgba=(0.018, 0.018, 0.016, 1.0))
    glass = model.material("fresnel_glass", rgba=(0.72, 0.88, 1.0, 0.42))
    gel_blue = model.material("blue_gel", rgba=(0.05, 0.20, 0.95, 0.48))

    lower_stand = model.part("lower_stand")
    lower_stand.visual(
        _save_mesh(_stand_mesh(), "tripod_lower_sleeve"),
        material=black_metal,
        name="tripod_lower_sleeve",
    )
    lower_stand.visual(
        Cylinder(radius=0.074, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=rubber,
        name="rubber_floor_pad",
    )

    center_column = model.part("center_column")
    center_column.visual(
        _save_mesh(_center_column_mesh(), "telescoping_column"),
        material=worn_steel,
        name="telescoping_tube",
    )
    center_column.visual(
        Cylinder(radius=0.090, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.791)),
        material=black_metal,
        name="pan_bearing_plate",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.086, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=black_metal,
        name="pan_bearing",
    )
    yoke.visual(
        Box((0.145, 0.720, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=black_metal,
        name="yoke_crossbar",
    )
    yoke.visual(
        Box((0.082, 0.062, 0.350)),
        origin=Origin(xyz=(0.0, -0.330, 0.292)),
        material=black_metal,
        name="yoke_arm_0",
    )
    yoke.visual(
        Cylinder(radius=0.050, length=0.064),
        origin=Origin(xyz=(0.0, -0.330, 0.390), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="tilt_boss_0",
    )
    yoke.visual(
        Box((0.082, 0.062, 0.350)),
        origin=Origin(xyz=(0.0, 0.330, 0.292)),
        material=black_metal,
        name="yoke_arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.050, length=0.064),
        origin=Origin(xyz=(0.0, 0.330, 0.390), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="tilt_boss_1",
    )

    can = model.part("can")
    can.visual(
        _save_mesh(_can_shell_mesh(), "spotlight_can_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.180, length=0.026),
        origin=Origin(xyz=(-0.352, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="rear_cap",
    )
    can.visual(
        Cylinder(radius=0.214, length=0.012),
        origin=Origin(xyz=(0.425, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="fresnel_lens",
    )
    can.visual(
        Cylinder(radius=0.030, length=0.580),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_shaft",
    )
    can.visual(
        Cylinder(radius=0.055, length=0.052),
        origin=Origin(xyz=(0.0, -0.272, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="trunnion_cap_0",
    )
    can.visual(
        Cylinder(radius=0.055, length=0.052),
        origin=Origin(xyz=(0.0, 0.272, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_metal,
        name="trunnion_cap_1",
    )
    can.visual(
        Box((0.040, 0.540, 0.012)),
        origin=Origin(xyz=(0.535, 0.0, 0.246)),
        material=black_metal,
        name="gel_guide_0",
    )
    can.visual(
        Box((0.040, 0.540, 0.012)),
        origin=Origin(xyz=(0.535, 0.0, 0.198)),
        material=black_metal,
        name="gel_guide_1",
    )
    can.visual(
        Box((0.040, 0.540, 0.012)),
        origin=Origin(xyz=(0.535, 0.0, -0.198)),
        material=black_metal,
        name="gel_guide_2",
    )
    can.visual(
        Box((0.040, 0.540, 0.012)),
        origin=Origin(xyz=(0.535, 0.0, -0.246)),
        material=black_metal,
        name="gel_guide_3",
    )
    can.visual(
        Box((0.026, 0.040, 0.066)),
        origin=Origin(xyz=(0.510, 0.0, 0.222)),
        material=black_metal,
        name="upper_guide_web",
    )
    can.visual(
        Box((0.026, 0.040, 0.066)),
        origin=Origin(xyz=(0.510, 0.0, -0.222)),
        material=black_metal,
        name="lower_guide_web",
    )

    gel_frame = model.part("gel_frame")
    gel_frame.visual(
        Box((0.016, 0.680, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=black_metal,
        name="top_bar",
    )
    gel_frame.visual(
        Box((0.016, 0.680, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=black_metal,
        name="bottom_bar",
    )
    for index, side in enumerate((-1.0, 1.0)):
        gel_frame.visual(
            Box((0.016, 0.030, 0.480)),
            origin=Origin(xyz=(0.0, side * 0.325, 0.0)),
            material=black_metal,
            name=f"side_bar_{index}",
        )
    gel_frame.visual(
        Box((0.004, 0.430, 0.430)),
        material=gel_blue,
        name="gel_sheet",
    )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=lower_stand,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=0.0, upper=0.45),
    )
    model.articulation(
        "yoke_pan",
        ArticulationType.CONTINUOUS,
        parent=center_column,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.802)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2),
    )
    model.articulation(
        "can_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.9, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "gel_slide",
        ArticulationType.PRISMATIC,
        parent=can,
        child=gel_frame,
        origin=Origin(xyz=(0.542, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.12, lower=0.0, upper=0.28),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_stand")
    column = object_model.get_part("center_column")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    gel = object_model.get_part("gel_frame")

    column_slide = object_model.get_articulation("column_slide")
    yoke_pan = object_model.get_articulation("yoke_pan")
    can_tilt = object_model.get_articulation("can_tilt")
    gel_slide = object_model.get_articulation("gel_slide")

    ctx.check(
        "primary theatre-light articulations are present",
        column_slide.articulation_type == ArticulationType.PRISMATIC
        and yoke_pan.articulation_type == ArticulationType.CONTINUOUS
        and can_tilt.articulation_type == ArticulationType.REVOLUTE
        and gel_slide.articulation_type == ArticulationType.PRISMATIC,
        details="Expected telescoping column, pan, tilt, and gel-frame slide joints.",
    )
    ctx.expect_within(
        column,
        lower,
        axes="xy",
        inner_elem="telescoping_tube",
        outer_elem="tripod_lower_sleeve",
        margin=0.005,
        name="center column stays concentric in lower sleeve",
    )
    ctx.expect_overlap(
        column,
        lower,
        axes="z",
        elem_a="telescoping_tube",
        elem_b="tripod_lower_sleeve",
        min_overlap=0.50,
        name="collapsed column remains deeply inserted",
    )

    rest_column_position = ctx.part_world_position(column)
    with ctx.pose({column_slide: 0.45}):
        ctx.expect_within(
            column,
            lower,
            axes="xy",
            inner_elem="telescoping_tube",
            outer_elem="tripod_lower_sleeve",
            margin=0.005,
            name="extended column stays concentric in sleeve",
        )
        ctx.expect_overlap(
            column,
            lower,
            axes="z",
            elem_a="telescoping_tube",
            elem_b="tripod_lower_sleeve",
            min_overlap=0.30,
            name="extended column keeps retained insertion",
        )
        extended_column_position = ctx.part_world_position(column)
    ctx.check(
        "column slide raises stand head",
        rest_column_position is not None
        and extended_column_position is not None
        and extended_column_position[2] > rest_column_position[2] + 0.40,
        details=f"rest={rest_column_position}, extended={extended_column_position}",
    )

    front_rest = ctx.part_element_world_aabb(can, elem="fresnel_lens")
    with ctx.pose({can_tilt: 0.80}):
        front_tilted = ctx.part_element_world_aabb(can, elem="fresnel_lens")
    ctx.check(
        "positive can tilt lifts the front lens",
        front_rest is not None and front_tilted is not None and front_tilted[0][2] > front_rest[0][2] + 0.08,
        details=f"rest={front_rest}, tilted={front_tilted}",
    )

    ctx.expect_overlap(
        gel,
        can,
        axes="y",
        elem_a="top_bar",
        elem_b="gel_guide_0",
        min_overlap=0.40,
        name="inserted gel frame spans top guide slot",
    )
    rest_gel_position = ctx.part_world_position(gel)
    with ctx.pose({gel_slide: 0.28}):
        ctx.expect_overlap(
            gel,
            can,
            axes="y",
            elem_a="top_bar",
            elem_b="gel_guide_0",
            min_overlap=0.20,
            name="pulled gel frame remains captured in guide",
        )
        pulled_gel_position = ctx.part_world_position(gel)
    ctx.check(
        "gel frame pulls out sideways",
        rest_gel_position is not None
        and pulled_gel_position is not None
        and pulled_gel_position[1] > rest_gel_position[1] + 0.25,
        details=f"rest={rest_gel_position}, pulled={pulled_gel_position}",
    )

    ctx.expect_contact(
        yoke,
        can,
        elem_a="tilt_boss_0",
        elem_b="trunnion_cap_0",
        contact_tol=0.020,
        name="can trunnion sits in yoke boss",
    )

    return ctx.report()


object_model = build_object_model()
