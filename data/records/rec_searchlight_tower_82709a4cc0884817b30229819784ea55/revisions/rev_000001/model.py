from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _build_lamp_shell():
    """Thin-walled searchlight housing, authored on local Z and later aimed along +Y."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.165, -0.230),
            (0.205, -0.165),
            (0.235, -0.030),
            (0.265, 0.165),
            (0.292, 0.360),
            (0.305, 0.425),
        ],
        [
            (0.115, -0.205),
            (0.166, -0.145),
            (0.192, -0.020),
            (0.218, 0.165),
            (0.244, 0.355),
            (0.252, 0.400),
        ],
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )


def _build_pivot_bushing():
    """Annular yoke bushing with a clear bore for the lamp trunnion shaft."""
    return LatheGeometry.from_shell_profiles(
        [(0.095, -0.0225), (0.095, 0.0225)],
        [(0.062, -0.0225), (0.062, 0.0225)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.58, 0.53, 0.45, 1.0))
    glass = model.material("pale_glass", rgba=(0.72, 0.86, 0.92, 0.42))
    black_rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.032, 1.0))

    fixed_support = model.part("fixed_support")
    fixed_support.visual(
        Cylinder(radius=0.34, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="base_plate",
    )
    fixed_support.visual(
        Cylinder(radius=0.120, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=painted_steel,
        name="mast_foot",
    )
    fixed_support.visual(
        Cylinder(radius=0.045, length=1.340),
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=painted_steel,
        name="mast_tube",
    )
    fixed_support.visual(
        Cylinder(radius=0.070, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=dark_steel,
        name="lower_collar",
    )
    fixed_support.visual(
        Cylinder(radius=0.070, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.090)),
        material=dark_steel,
        name="upper_collar",
    )
    fixed_support.visual(
        Cylinder(radius=0.130, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 1.500)),
        material=dark_steel,
        name="top_bearing",
    )

    brace_sets = [
        ((0.280, 0.000, 0.075), (0.038, 0.000, 0.850)),
        ((-0.280, 0.000, 0.075), (-0.038, 0.000, 0.850)),
        ((0.000, 0.280, 0.075), (0.000, 0.038, 0.850)),
        ((0.000, -0.280, 0.075), (0.000, -0.038, 0.850)),
    ]
    for index, points in enumerate(brace_sets):
        fixed_support.visual(
            _save_mesh(
                tube_from_spline_points(
                    list(points),
                    radius=0.016,
                    samples_per_segment=4,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"tower_brace_{index}",
            ),
            material=painted_steel,
            name=f"brace_{index}",
        )

    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.158, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_steel,
        name="bearing_drum",
    )
    pan_carriage.visual(
        Cylinder(radius=0.260, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=painted_steel,
        name="rotating_deck",
    )
    pan_carriage.visual(
        Cylinder(radius=0.105, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=painted_steel,
        name="turret_neck",
    )
    pan_carriage.visual(
        _save_mesh(
            TrunnionYokeGeometry(
                (0.860, 0.220, 0.620),
                span_width=0.640,
                trunnion_diameter=0.120,
                trunnion_center_z=0.430,
                base_thickness=0.120,
                corner_radius=0.025,
                center=False,
            ),
            "searchlight_yoke_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=painted_steel,
        name="yoke_frame",
    )
    pan_carriage.visual(
        _save_mesh(_build_pivot_bushing(), "pivot_bushing_0"),
        origin=Origin(xyz=(0.452, 0.0, 0.567), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="pivot_bushing_0",
    )
    pan_carriage.visual(
        _save_mesh(_build_pivot_bushing(), "pivot_bushing_1"),
        origin=Origin(xyz=(-0.452, 0.0, 0.567), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="pivot_bushing_1",
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        _save_mesh(_build_lamp_shell(), "searchlight_lamp_shell"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="lamp_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.045, length=0.990),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    lamp_head.visual(
        Cylinder(radius=0.108, length=0.050),
        origin=Origin(xyz=(0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="side_hub_0",
    )
    lamp_head.visual(
        Cylinder(radius=0.108, length=0.050),
        origin=Origin(xyz=(-0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="side_hub_1",
    )
    lamp_head.visual(
        Cylinder(radius=0.246, length=0.018),
        origin=Origin(xyz=(0.0, 0.418, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp_head.visual(
        _save_mesh(
            TorusGeometry(radius=0.270, tube=0.030, radial_segments=18, tubular_segments=72),
            "searchlight_front_lip",
        ),
        origin=Origin(xyz=(0.0, 0.418, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_lip",
    )
    lamp_head.visual(
        Box((0.420, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.430, 0.0)),
        material=dark_steel,
        name="lens_bar_x",
    )
    lamp_head.visual(
        Box((0.018, 0.014, 0.420)),
        origin=Origin(xyz=(0.0, 0.431, 0.0)),
        material=dark_steel,
        name="lens_bar_z",
    )
    lamp_head.visual(
        Cylinder(radius=0.155, length=0.050),
        origin=Origin(xyz=(0.0, -0.246, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_cap",
    )
    lamp_head.visual(
        _save_mesh(
            tube_from_spline_points(
                [
                    (-0.150, -0.030, 0.170),
                    (-0.075, 0.080, 0.330),
                    (0.075, 0.080, 0.330),
                    (0.150, -0.030, 0.170),
                ],
                radius=0.014,
                samples_per_segment=12,
                radial_segments=14,
                cap_ends=True,
            ),
            "searchlight_top_handle",
        ),
        material=black_rubber,
        name="top_handle",
    )
    lamp_head.visual(
        Cylinder(radius=0.078, length=0.045),
        origin=Origin(xyz=(0.497, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="retaining_cap_0",
    )
    lamp_head.visual(
        Cylinder(radius=0.078, length=0.045),
        origin=Origin(xyz=(-0.497, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="retaining_cap_1",
    )

    model.articulation(
        "base_pan",
        ArticulationType.REVOLUTE,
        parent=fixed_support,
        child=pan_carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.567)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.9, lower=-0.45, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("fixed_support")
    pan = object_model.get_part("pan_carriage")
    lamp = object_model.get_part("lamp_head")
    pan_joint = object_model.get_articulation("base_pan")
    tilt_joint = object_model.get_articulation("yoke_tilt")

    ctx.expect_gap(
        pan,
        support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="bearing_drum",
        negative_elem="top_bearing",
        name="pan bearing sits on mast support",
    )
    ctx.expect_within(
        lamp,
        pan,
        axes="x",
        inner_elem="lamp_shell",
        outer_elem="yoke_frame",
        margin=0.0,
        name="lamp body is held between yoke arms",
    )

    def _coord(vec, index: int) -> float:
        return (vec.x, vec.y, vec.z)[index] if hasattr(vec, "x") else vec[index]

    def _extent(aabb, index: int) -> float:
        return _coord(aabb[1], index) - _coord(aabb[0], index)

    yoke_aabb = ctx.part_element_world_aabb(pan, elem="yoke_frame")
    bearing_aabb = ctx.part_element_world_aabb(support, elem="top_bearing")
    shell_aabb = ctx.part_element_world_aabb(lamp, elem="lamp_shell")
    ctx.check(
        "rotating yoke is large relative to fixed support",
        yoke_aabb is not None
        and bearing_aabb is not None
        and _extent(yoke_aabb, 0) > 2.8 * _extent(bearing_aabb, 0),
        details=f"yoke_aabb={yoke_aabb}, bearing_aabb={bearing_aabb}",
    )
    ctx.check(
        "lamp shell clears the yoke span",
        shell_aabb is not None and _extent(shell_aabb, 0) < 0.620,
        details=f"lamp_shell_aabb={shell_aabb}",
    )

    rest_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({tilt_joint: 0.85}):
        raised_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    rest_center_z = None if rest_lens is None else (_coord(rest_lens[0], 2) + _coord(rest_lens[1], 2)) / 2.0
    raised_center_z = None if raised_lens is None else (_coord(raised_lens[0], 2) + _coord(raised_lens[1], 2)) / 2.0
    ctx.check(
        "positive tilt raises the searchlight beam",
        rest_center_z is not None and raised_center_z is not None and raised_center_z > rest_center_z + 0.20,
        details=f"rest_lens={rest_lens}, raised_lens={raised_lens}",
    )

    rest_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    rest_center_y = None if rest_lens is None else (_coord(rest_lens[0], 1) + _coord(rest_lens[1], 1)) / 2.0
    panned_center_x = None if panned_lens is None else (_coord(panned_lens[0], 0) + _coord(panned_lens[1], 0)) / 2.0
    ctx.check(
        "pan joint swings the lamp around the vertical mast",
        rest_center_y is not None and panned_center_x is not None and abs(panned_center_x) > rest_center_y * 0.75,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
