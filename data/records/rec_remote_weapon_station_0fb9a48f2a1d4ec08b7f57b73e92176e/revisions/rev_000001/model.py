from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)
def _side_lug(y_center: float) -> MeshGeometry:
    """Connected rectangular side lug with a clear trunnion opening."""
    lug = MeshGeometry()
    for x, z, sx, sz in (
        (-0.105, 0.34, 0.070, 0.360),
        (0.105, 0.34, 0.070, 0.360),
        (0.0, 0.195, 0.280, 0.070),
        (0.0, 0.485, 0.280, 0.070),
    ):
        lug.merge(BoxGeometry((sx, 0.070, sz)).translate(x, y_center, z))
    return lug


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_remote_weapon_station")

    model.material("olive_drab", rgba=(0.25, 0.32, 0.22, 1.0))
    model.material("dark_armor", rgba=(0.10, 0.12, 0.11, 1.0))
    model.material("gunmetal", rgba=(0.06, 0.065, 0.07, 1.0))
    model.material("worn_steel", rgba=(0.48, 0.50, 0.47, 1.0))
    model.material("black_glass", rgba=(0.02, 0.03, 0.035, 0.75))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.26, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material="dark_armor",
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.155, length=0.672),
        origin=Origin(xyz=(0.0, 0.0, 0.364)),
        material="olive_drab",
        name="pedestal_column",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.34, tube=0.035, radial_segments=18, tubular_segments=64), "fixed_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material="worn_steel",
        name="fixed_ring",
    )
    pedestal.visual(
        Box((0.76, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        material="olive_drab",
        name="ring_spoke_x",
    )
    pedestal.visual(
        Box((0.76, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.675), rpy=(0.0, 0.0, pi / 2.0)),
        material="olive_drab",
        name="ring_spoke_y",
    )

    azimuth_frame = model.part("azimuth_frame")
    azimuth_frame.visual(
        Cylinder(radius=0.255, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="dark_armor",
        name="turntable",
    )
    azimuth_frame.visual(
        Cylinder(radius=0.115, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material="worn_steel",
        name="azimuth_collar",
    )
    for x in (-0.13, 0.13):
        for y in (-0.29, 0.29):
            azimuth_frame.visual(
                Box((0.052, 0.052, 0.6575)),
                origin=Origin(xyz=(x, y, 0.36375)),
                material="olive_drab",
                name=f"roof_post_{'f' if x > 0 else 'r'}_{'p' if y > 0 else 'n'}",
            )
    azimuth_frame.visual(
        mesh_from_geometry(_side_lug(0.24), "side_lug_0"),
        material="olive_drab",
        name="side_lug_0",
    )
    azimuth_frame.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(xyz=(0.0, 0.24, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_steel",
        name="trunnion_bushing_0",
    )
    azimuth_frame.visual(
        Box((0.105, 0.105, 0.130)),
        origin=Origin(xyz=(0.0, 0.264, 0.095)),
        material="olive_drab",
        name="lug_stanchion_0",
    )
    azimuth_frame.visual(
        mesh_from_geometry(_side_lug(-0.24), "side_lug_1"),
        material="olive_drab",
        name="side_lug_1",
    )
    azimuth_frame.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(xyz=(0.0, -0.24, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_steel",
        name="trunnion_bushing_1",
    )
    azimuth_frame.visual(
        Box((0.105, 0.105, 0.130)),
        origin=Origin(xyz=(0.0, -0.264, 0.095)),
        material="olive_drab",
        name="lug_stanchion_1",
    )
    azimuth_frame.visual(
        Box((0.72, 0.72, 0.035)),
        origin=Origin(xyz=(0.08, 0.0, 0.710)),
        material="olive_drab",
        name="roof_panel",
    )
    azimuth_frame.visual(
        Box((0.065, 0.72, 0.175)),
        origin=Origin(xyz=(0.425, 0.0, 0.642), rpy=(0.0, -0.24, 0.0)),
        material="olive_drab",
        name="front_roof_lip",
    )
    azimuth_frame.visual(
        Box((0.62, 0.045, 0.115)),
        origin=Origin(xyz=(0.055, 0.375, 0.655)),
        material="olive_drab",
        name="side_roof_lip_0",
    )
    azimuth_frame.visual(
        Box((0.62, 0.045, 0.115)),
        origin=Origin(xyz=(0.055, -0.375, 0.655)),
        material="olive_drab",
        name="side_roof_lip_1",
    )
    azimuth_frame.visual(
        Box((0.070, 0.155, 0.052)),
        origin=Origin(xyz=(0.11, -0.3525, 0.418)),
        material="olive_drab",
        name="sight_bracket",
    )
    azimuth_frame.visual(
        Cylinder(radius=0.032, length=0.075),
        origin=Origin(xyz=(0.11, -0.43, 0.47), rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_steel",
        name="sight_bushing",
    )

    gun_cradle = model.part("gun_cradle")
    gun_cradle.visual(
        Cylinder(radius=0.040, length=0.550),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_steel",
        name="trunnion_pin",
    )
    gun_cradle.visual(
        Box((0.420, 0.200, 0.150)),
        origin=Origin(xyz=(0.050, 0.0, -0.030)),
        material="gunmetal",
        name="receiver",
    )
    gun_cradle.visual(
        Box((0.150, 0.160, 0.115)),
        origin=Origin(xyz=(-0.225, 0.0, -0.020)),
        material="dark_armor",
        name="rear_counterweight",
    )
    gun_cradle.visual(
        Box((0.250, 0.160, 0.100)),
        origin=Origin(xyz=(0.025, 0.0, -0.145)),
        material="dark_armor",
        name="ammo_box",
    )
    gun_cradle.visual(
        Cylinder(radius=0.045, length=0.235),
        origin=Origin(xyz=(0.365, 0.0, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_armor",
        name="barrel_shroud",
    )
    gun_cradle.visual(
        Cylinder(radius=0.026, length=0.790),
        origin=Origin(xyz=(0.680, 0.0, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material="gunmetal",
        name="barrel",
    )
    gun_cradle.visual(
        Cylinder(radius=0.038, length=0.075),
        origin=Origin(xyz=(1.095, 0.0, 0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_armor",
        name="muzzle_brake",
    )
    sight_unit = model.part("sight_unit")
    sight_unit.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="worn_steel",
        name="sight_pivot",
    )
    sight_unit.visual(
        Box((0.075, 0.055, 0.070)),
        origin=Origin(xyz=(0.040, -0.0875, 0.010)),
        material="dark_armor",
        name="pivot_housing",
    )
    sight_unit.visual(
        Box((0.185, 0.120, 0.125)),
        origin=Origin(xyz=(0.145, -0.155, 0.040)),
        material="dark_armor",
        name="optic_body",
    )
    sight_unit.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.2475, -0.155, 0.040), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_glass",
        name="front_lens",
    )

    model.articulation(
        "base_azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=azimuth_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.4),
    )
    model.articulation(
        "cradle_elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth_frame,
        child=gun_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.25, upper=0.45, effort=180.0, velocity=1.0),
    )
    model.articulation(
        "sight_tilt",
        ArticulationType.REVOLUTE,
        parent=azimuth_frame,
        child=sight_unit,
        origin=Origin(xyz=(0.11, -0.43, 0.47)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=15.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    azimuth = object_model.get_articulation("base_azimuth")
    elevation = object_model.get_articulation("cradle_elevation")
    sight_tilt = object_model.get_articulation("sight_tilt")
    frame = object_model.get_part("azimuth_frame")
    cradle = object_model.get_part("gun_cradle")
    sight = object_model.get_part("sight_unit")

    ctx.allow_overlap(
        frame,
        cradle,
        elem_a="trunnion_bushing_0",
        elem_b="trunnion_pin",
        reason="The trunnion pin is intentionally represented as captured inside the solid bushing proxy of the side lug.",
    )
    ctx.allow_overlap(
        frame,
        cradle,
        elem_a="trunnion_bushing_1",
        elem_b="trunnion_pin",
        reason="The trunnion pin is intentionally represented as captured inside the solid bushing proxy of the side lug.",
    )
    ctx.allow_overlap(
        frame,
        sight,
        elem_a="sight_bushing",
        elem_b="sight_pivot",
        reason="The small sight tilt shaft is intentionally seated inside the side bushing proxy.",
    )

    ctx.expect_within(
        cradle,
        frame,
        axes="xz",
        inner_elem="trunnion_pin",
        outer_elem="side_lug_0",
        margin=0.002,
        name="trunnion pin centered in upper side lug",
    )
    ctx.expect_within(
        cradle,
        frame,
        axes="xz",
        inner_elem="trunnion_pin",
        outer_elem="side_lug_1",
        margin=0.002,
        name="trunnion pin centered in lower side lug",
    )
    ctx.expect_overlap(
        cradle,
        frame,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="side_lug_0",
        min_overlap=0.045,
        name="pin remains inserted in side lug 0",
    )
    ctx.expect_overlap(
        cradle,
        frame,
        axes="y",
        elem_a="trunnion_pin",
        elem_b="side_lug_1",
        min_overlap=0.045,
        name="pin remains inserted in side lug 1",
    )
    ctx.expect_within(
        cradle,
        frame,
        axes="xz",
        inner_elem="trunnion_pin",
        outer_elem="trunnion_bushing_0",
        margin=0.001,
        name="pin centered in bushing 0",
    )
    ctx.expect_within(
        cradle,
        frame,
        axes="xz",
        inner_elem="trunnion_pin",
        outer_elem="trunnion_bushing_1",
        margin=0.001,
        name="pin centered in bushing 1",
    )
    ctx.expect_overlap(
        sight,
        frame,
        axes="y",
        elem_a="sight_pivot",
        elem_b="sight_bushing",
        min_overlap=0.060,
        name="sight pivot is carried by frame bushing",
    )
    ctx.expect_within(
        sight,
        frame,
        axes="xz",
        inner_elem="sight_pivot",
        outer_elem="sight_bushing",
        margin=0.001,
        name="sight pivot stays centered in bushing",
    )

    def _aabb_center_z(part, elem: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return (lo[2] + hi[2]) / 2.0

    def _aabb_center_y(part, elem: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return (lo[1] + hi[1]) / 2.0

    muzzle_z_rest = _aabb_center_z(cradle, "muzzle_brake")
    muzzle_y_rest = _aabb_center_y(cradle, "muzzle_brake")
    with ctx.pose({elevation: 0.45}):
        muzzle_z_high = _aabb_center_z(cradle, "muzzle_brake")
        ctx.expect_overlap(
            cradle,
            frame,
            axes="y",
            elem_a="trunnion_pin",
            elem_b="side_lug_0",
            min_overlap=0.045,
            name="raised gun stays captured by side lug 0",
        )
        ctx.expect_overlap(
            cradle,
            frame,
            axes="y",
            elem_a="trunnion_pin",
            elem_b="side_lug_1",
            min_overlap=0.045,
            name="raised gun stays captured by side lug 1",
        )
    ctx.check(
        "positive elevation raises muzzle",
        muzzle_z_rest is not None and muzzle_z_high is not None and muzzle_z_high > muzzle_z_rest + 0.12,
        details=f"rest_z={muzzle_z_rest}, raised_z={muzzle_z_high}",
    )

    sight_z_rest = _aabb_center_z(sight, "optic_body")
    with ctx.pose({sight_tilt: 0.35}):
        sight_z_high = _aabb_center_z(sight, "optic_body")
    ctx.check(
        "sight tilt raises optic",
        sight_z_rest is not None and sight_z_high is not None and sight_z_high > sight_z_rest + 0.015,
        details=f"rest_z={sight_z_rest}, tilted_z={sight_z_high}",
    )

    with ctx.pose({azimuth: pi / 2.0}):
        muzzle_y_rotated = _aabb_center_y(cradle, "muzzle_brake")
    ctx.check(
        "azimuth rotates weapon about vertical ring axis",
        muzzle_y_rest is not None and muzzle_y_rotated is not None and muzzle_y_rotated > muzzle_y_rest + 0.85,
        details=f"rest_y={muzzle_y_rest}, rotated_y={muzzle_y_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
