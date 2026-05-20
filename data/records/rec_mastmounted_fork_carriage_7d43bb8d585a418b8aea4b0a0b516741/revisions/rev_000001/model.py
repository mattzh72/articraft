from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)
import cadquery as cq


def _tapered_fork_geometry(
    length: float,
    width: float,
    rear_height: float,
    tip_height: float,
) -> MeshGeometry:
    """Rectangular fork tine with a machined taper from heel to tip."""
    half = width / 2.0
    vertices = [
        (-half, 0.0, 0.0),
        (half, 0.0, 0.0),
        (half, length, 0.0),
        (-half, length, 0.0),
        (-half, 0.0, rear_height),
        (half, 0.0, rear_height),
        (half, length, tip_height),
        (-half, length, tip_height),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (0, 3, 7),
        (0, 7, 4),
        (1, 5, 6),
        (1, 6, 2),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def _triangular_gusset_geometry(
    thickness: float,
    points_yz: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
) -> MeshGeometry:
    """Triangular plate in the YZ plane, with thickness along X."""
    half = thickness / 2.0
    vertices = [(-half, y, z) for y, z in points_yz] + [(half, y, z) for y, z in points_yz]
    faces = [
        (0, 1, 2),
        (3, 5, 4),
        (0, 3, 4),
        (0, 4, 1),
        (1, 4, 5),
        (1, 5, 2),
        (2, 5, 3),
        (2, 3, 0),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def _add_box(part, name: str, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cylinder(part, name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_mounted_fork_carriage_study")

    hot_rolled = model.material("hot_rolled_steel", rgba=(0.23, 0.25, 0.26, 1.0))
    dark_oxide = model.material("black_oxide_steel", rgba=(0.05, 0.055, 0.055, 1.0))
    machined = model.material("machined_bright_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    wear_bronze = model.material("bronze_wear_surface", rgba=(0.63, 0.45, 0.18, 1.0))
    zinc = model.material("zinc_fasteners", rgba=(0.72, 0.72, 0.68, 1.0))
    polymer = model.material("dark_guide_shoe", rgba=(0.025, 0.026, 0.025, 1.0))
    cover_gray = model.material("removable_cover_gray", rgba=(0.34, 0.36, 0.36, 1.0))

    mast = model.part("mast")

    # Stationary welded mast: two C-channel uprights with top/bottom cross heads,
    # rear service spine, track strips, and visible chain pulley support blocks.
    for sx in (-1.0, 1.0):
        x = sx * 0.38
        lip_x = sx * 0.34
        _add_box(mast, f"outer_web_{sx:+.0f}", (0.045, 0.120, 2.20), (x, 0.0, 1.10), hot_rolled)
        _add_box(mast, f"outer_front_lip_{sx:+.0f}", (0.035, 0.030, 2.20), (lip_x, 0.045, 1.10), hot_rolled)
        _add_box(mast, f"outer_rear_lip_{sx:+.0f}", (0.035, 0.030, 2.20), (lip_x, -0.045, 1.10), hot_rolled)
        _add_box(mast, f"outer_wear_strip_{sx:+.0f}", (0.018, 0.010, 1.94), (sx * 0.315, 0.060, 1.12), wear_bronze)
        _add_box(mast, f"base_foot_{sx:+.0f}", (0.26, 0.34, 0.035), (sx * 0.38, 0.0, 0.0175), hot_rolled)
        _add_cylinder(
            mast,
            f"floor_anchor_{sx:+.0f}_0",
            0.018,
            0.012,
            (sx * 0.38, 0.115, 0.041),
            zinc,
        )
        _add_cylinder(
            mast,
            f"floor_anchor_{sx:+.0f}_1",
            0.018,
            0.012,
            (sx * 0.38, -0.115, 0.041),
            zinc,
        )

    _add_box(mast, "bottom_crosshead", (0.88, 0.150, 0.080), (0.0, 0.0, 0.040), hot_rolled)
    _add_box(mast, "top_crosshead", (0.88, 0.150, 0.085), (0.0, 0.0, 2.2425), hot_rolled)
    _add_box(mast, "rear_service_spine", (0.18, 0.018, 2.12), (0.0, -0.075, 1.14), hot_rolled)
    _add_box(mast, "chain_pulley_beam", (0.36, 0.080, 0.070), (0.0, 0.025, 2.165), dark_oxide)
    _add_cylinder(mast, "upper_chain_sheave", 0.060, 0.060, (0.0, 0.045, 2.165), machined, rpy=(0.0, math.pi / 2.0, 0.0))
    _add_cylinder(mast, "sheave_pin", 0.016, 0.44, (0.0, 0.045, 2.165), zinc, rpy=(0.0, math.pi / 2.0, 0.0))

    inner_channel = model.part("inner_channel")
    for sx in (-1.0, 1.0):
        x = sx * 0.255
        lip_x = sx * 0.225
        _add_box(inner_channel, f"inner_web_{sx:+.0f}", (0.040, 0.095, 1.70), (x, 0.0, 0.85), dark_oxide)
        _add_box(inner_channel, f"inner_front_lip_{sx:+.0f}", (0.030, 0.025, 1.70), (lip_x, 0.0375, 0.85), dark_oxide)
        _add_box(inner_channel, f"inner_rear_lip_{sx:+.0f}", (0.030, 0.025, 1.70), (lip_x, -0.0375, 0.85), dark_oxide)
        _add_box(inner_channel, f"roller_track_{sx:+.0f}", (0.070, 0.010, 1.54), (x, 0.050, 0.86), machined)
    _add_box(inner_channel, "inner_bottom_tie", (0.58, 0.060, 0.055), (0.0, -0.010, 0.055), dark_oxide)
    _add_box(inner_channel, "inner_top_tie", (0.58, 0.060, 0.060), (0.0, -0.010, 1.675), dark_oxide)
    _add_box(inner_channel, "inner_chain_anchor_pad", (0.14, 0.035, 0.090), (0.0, 0.020, 1.60), dark_oxide)

    model.articulation(
        "mast_to_inner",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=inner_channel,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.35, lower=0.0, upper=0.55),
        motion_properties=MotionProperties(damping=80.0, friction=35.0),
    )

    carriage = model.part("carriage")

    # Fork carriage frame: rigid welded plate-and-tube structure, backrest grid,
    # replaceable guide shoes, roller brackets, chain clevises, fork heels, and gussets.
    _add_box(carriage, "lower_crossbar", (0.74, 0.105, 0.090), (0.0, 0.145, 0.020), hot_rolled)
    _add_box(carriage, "middle_crossbar", (0.70, 0.085, 0.070), (0.0, 0.142, 0.405), hot_rolled)
    _add_box(carriage, "upper_crossbar", (0.74, 0.095, 0.075), (0.0, 0.142, 0.800), hot_rolled)
    for sx in (-1.0, 1.0):
        _add_box(carriage, f"side_upright_{sx:+.0f}", (0.060, 0.095, 0.870), (sx * 0.330, 0.142, 0.390), hot_rolled)
        _add_box(carriage, f"fork_hook_plate_{sx:+.0f}", (0.085, 0.120, 0.200), (sx * 0.220, 0.202, -0.020), hot_rolled)
        _add_box(carriage, f"guide_shoe_upper_{sx:+.0f}", (0.095, 0.040, 0.110), (sx * 0.260, 0.075, 0.500), polymer)
        _add_box(carriage, f"guide_shoe_lower_{sx:+.0f}", (0.095, 0.040, 0.110), (sx * 0.260, 0.075, 0.310), polymer)

        # Roller yokes are fixed to the carriage and carry separate spinning rollers.
        for zi, z in enumerate((0.170, 0.690)):
            _add_box(carriage, f"roller_base_{sx:+.0f}_{zi}", (0.120, 0.022, 0.130), (sx * 0.270, 0.157, z), hot_rolled)
            _add_box(carriage, f"roller_cheek_in_{sx:+.0f}_{zi}", (0.007, 0.060, 0.130), (sx * 0.246, 0.127, z), hot_rolled)
            _add_box(carriage, f"roller_cheek_out_{sx:+.0f}_{zi}", (0.007, 0.060, 0.130), (sx * 0.294, 0.127, z), hot_rolled)

    fork_mesh = mesh_from_geometry(_tapered_fork_geometry(0.96, 0.105, 0.075, 0.032), "tapered_fork_tine")
    for sx in (-1.0, 1.0):
        carriage.visual(
            fork_mesh,
            origin=Origin(xyz=(sx * 0.210, 0.260, -0.150)),
            material=hot_rolled,
            name=f"fork_tine_{sx:+.0f}",
        )
        gusset_mesh = mesh_from_geometry(
            _triangular_gusset_geometry(0.020, ((0.0, 0.0), (0.0, 0.310), (0.300, 0.0))),
            f"heel_gusset_{sx:+.0f}",
        )
        carriage.visual(
            gusset_mesh,
            origin=Origin(xyz=(sx * 0.210, 0.255, -0.105)),
            material=hot_rolled,
            name=f"heel_gusset_{sx:+.0f}",
        )

    # Load backrest grid, intentionally workmanlike and fabricated rather than styled.
    for x in (-0.33, -0.11, 0.11, 0.33):
        _add_box(carriage, f"backrest_post_{x:+.2f}", (0.035, 0.052, 0.620), (x, 0.110, 1.090), hot_rolled)
    for z in (0.920, 1.130, 1.330):
        _add_box(carriage, f"backrest_slat_{z:.2f}", (0.760, 0.050, 0.045), (0.0, 0.110, z), hot_rolled)
    _add_box(carriage, "chain_anchor_base", (0.300, 0.070, 0.035), (0.0, 0.085, 0.870), hot_rolled)
    clevis_mesh = mesh_from_geometry(
        ClevisBracketGeometry(
            (0.095, 0.070, 0.115),
            gap_width=0.034,
            bore_diameter=0.018,
            bore_center_z=0.074,
            base_thickness=0.018,
            corner_radius=0.006,
            center=False,
        ),
        "chain_anchor_clevis",
    )
    for x in (-0.070, 0.070):
        carriage.visual(clevis_mesh, origin=Origin(xyz=(x, 0.086, 0.887)), material=dark_oxide, name=f"chain_clevis_{x:+.2f}")
        _add_cylinder(carriage, f"chain_pin_{x:+.2f}", 0.011, 0.090, (x, 0.086, 0.961), zinc, rpy=(math.pi / 2.0, 0.0, 0.0))
        _add_box(carriage, f"chain_pin_keeper_{x:+.2f}", (0.018, 0.040, 0.080), (x, 0.086, 0.925), zinc)

    model.articulation(
        "inner_to_carriage",
        ArticulationType.PRISMATIC,
        parent=inner_channel,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.32, lower=0.0, upper=0.65),
        motion_properties=MotionProperties(damping=90.0, friction=45.0),
    )

    roller_positions = [
        (-0.270, 0.100, 0.170),
        (0.270, 0.100, 0.170),
        (-0.270, 0.100, 0.690),
        (0.270, 0.100, 0.690),
    ]
    for index, xyz in enumerate(roller_positions):
        roller = model.part(f"roller_{index}")
        _add_cylinder(roller, "roller_body", 0.045, 0.036, (0.0, 0.0, 0.0), machined, rpy=(0.0, math.pi / 2.0, 0.0))
        _add_cylinder(roller, "bearing_hub", 0.020, 0.034, (0.0, 0.0, 0.0), zinc, rpy=(0.0, math.pi / 2.0, 0.0))
        _add_cylinder(roller, "side_flange_0", 0.034, 0.004, (-0.020, 0.0, 0.0), dark_oxide, rpy=(0.0, math.pi / 2.0, 0.0))
        _add_cylinder(roller, "side_flange_1", 0.034, 0.004, (0.020, 0.0, 0.0), dark_oxide, rpy=(0.0, math.pi / 2.0, 0.0))
        model.articulation(
            f"carriage_to_roller_{index}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=roller,
            origin=Origin(xyz=xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=18.0),
            motion_properties=MotionProperties(damping=0.03, friction=0.02),
        )

    # Removable rear access covers are separate fixed parts with visible screws.
    for index, z in enumerate((0.72, 1.38)):
        cover = model.part(f"cover_{index}")
        _add_box(cover, "cover_plate", (0.160, 0.012, 0.360), (0.0, 0.0, 0.0), cover_gray)
        for x in (-0.055, 0.055):
            for bolt_z in (-0.135, 0.135):
                _add_cylinder(
                    cover,
                    f"cover_bolt_{x:+.2f}_{bolt_z:+.2f}",
                    0.009,
                    0.008,
                    (x, -0.010, bolt_z),
                    zinc,
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                )
        model.articulation(
            f"mast_to_cover_{index}",
            ArticulationType.FIXED,
            parent=mast,
            child=cover,
            origin=Origin(xyz=(0.0, -0.090, z)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    inner_channel = object_model.get_part("inner_channel")
    carriage = object_model.get_part("carriage")
    cover_0 = object_model.get_part("cover_0")
    inner_lift = object_model.get_articulation("mast_to_inner")
    carriage_lift = object_model.get_articulation("inner_to_carriage")

    ctx.check(
        "two vertical lift joints are present",
        inner_lift is not None and carriage_lift is not None,
        details=f"inner_lift={inner_lift}, carriage_lift={carriage_lift}",
    )

    roller_joints = [object_model.get_articulation(f"carriage_to_roller_{i}") for i in range(4)]
    ctx.check(
        "four exposed roller bearing joints are present",
        all(joint is not None for joint in roller_joints),
        details=f"roller_joints={roller_joints}",
    )

    # The lower rollers sit tangent to the inner mast tracks and overlap in
    # projection, proving the carriage has a visible guided lift path.
    for index in (0, 1):
        roller = object_model.get_part(f"roller_{index}")
        ctx.expect_gap(
            roller,
            inner_channel,
            axis="y",
            max_gap=0.003,
            max_penetration=0.001,
            name=f"roller_{index} bears on inner channel track",
        )
        ctx.expect_overlap(
            roller,
            inner_channel,
            axes="xz",
            min_overlap=0.025,
            name=f"roller_{index} aligns with track",
        )

    ctx.expect_contact(
        mast,
        cover_0,
        contact_tol=0.001,
        name="removable cover sits on rear service spine",
    )

    inner_rest = ctx.part_world_position(inner_channel)
    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({inner_lift: 0.55, carriage_lift: 0.65}):
        inner_raised = ctx.part_world_position(inner_channel)
        carriage_raised = ctx.part_world_position(carriage)

    ctx.check(
        "nested channel lifts upward",
        inner_rest is not None and inner_raised is not None and inner_raised[2] > inner_rest[2] + 0.50,
        details=f"rest={inner_rest}, raised={inner_raised}",
    )
    ctx.check(
        "fork carriage follows clear vertical lift path",
        carriage_rest is not None and carriage_raised is not None and carriage_raised[2] > carriage_rest[2] + 1.10,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )

    return ctx.report()


object_model = build_object_model()
