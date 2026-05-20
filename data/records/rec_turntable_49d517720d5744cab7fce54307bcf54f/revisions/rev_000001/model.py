from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_turntable")

    powdercoat = model.material("powdercoat_graphite", rgba=(0.15, 0.16, 0.17, 1.0))
    dark_rubber = model.material("seal_black", rgba=(0.08, 0.08, 0.08, 1.0))
    stainless = model.material("stainless_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.60, 0.61, 0.63, 1.0))
    polymer = model.material("engineering_polymer", rgba=(0.19, 0.20, 0.21, 1.0))

    def add_x_cylinder(part, *, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.44, 0.32, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=powdercoat,
        name="anchor_skirt",
    )
    plinth.visual(
        Box((0.48, 0.36, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=powdercoat,
        name="sealed_body",
    )
    plinth.visual(
        Box((0.54, 0.42, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=powdercoat,
        name="rain_cap",
    )
    plinth.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
        material=polymer,
        name="platter_bearing_shroud",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=dark_rubber,
        name="spindle_seal",
    )
    plinth.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=stainless,
        name="thrust_pad",
    )
    plinth.visual(
        Cylinder(radius=0.050, length=0.026),
        origin=Origin(xyz=(0.198, -0.060, 0.181)),
        material=powdercoat,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.198, -0.060, 0.190)),
        material=dark_rubber,
        name="joint_weather_skirt",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=0.002),
        origin=Origin(xyz=(0.198, -0.060, 0.195)),
        material=stainless,
        name="bearing_cap",
    )

    for index, (x_pos, y_pos) in enumerate(
        ((0.230, 0.170), (-0.230, 0.170), (-0.230, -0.170), (0.230, -0.170)),
        start=1,
    ):
        plinth.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.171)),
            material=stainless,
            name=f"anchor_bolt_{index}",
        )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=stainless,
        name="spindle_tip",
    )
    platter.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=stainless,
        name="hub",
    )
    platter.visual(
        Cylinder(radius=0.155, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=satin_aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.148, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=dark_rubber,
        name="platter_mat",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=stainless,
        name="record_spindle",
    )

    tonearm = model.part("tonearm_stage")
    tonearm.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stainless,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.042, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=powdercoat,
        name="armboard",
    )
    tonearm.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=powdercoat,
        name="weather_dome",
    )
    add_x_cylinder(
        tonearm,
        name="arm_hub",
        radius=0.012,
        length=0.028,
        xyz=(-0.014, 0.0, 0.045),
        material=stainless,
    )
    add_x_cylinder(
        tonearm,
        name="arm_tube",
        radius=0.0055,
        length=0.210,
        xyz=(-0.133, 0.0, 0.045),
        material=satin_aluminum,
    )
    tonearm.visual(
        Box((0.028, 0.018, 0.010)),
        origin=Origin(xyz=(-0.252, 0.0, 0.045)),
        material=powdercoat,
        name="headshell",
    )
    tonearm.visual(
        Box((0.013, 0.012, 0.010)),
        origin=Origin(xyz=(-0.255, 0.0, 0.035)),
        material=polymer,
        name="cartridge",
    )
    add_x_cylinder(
        tonearm,
        name="counterweight_stub",
        radius=0.006,
        length=0.052,
        xyz=(0.026, 0.0, 0.045),
        material=stainless,
    )
    add_x_cylinder(
        tonearm,
        name="counterweight",
        radius=0.018,
        length=0.030,
        xyz=(0.067, 0.0, 0.045),
        material=satin_aluminum,
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.198, -0.060, 0.196)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm_stage")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "articulation_definitions_present",
        platter_joint.axis == (0.0, 0.0, 1.0)
        and tonearm_joint.axis == (0.0, 0.0, 1.0)
        and platter_joint.motion_limits is not None
        and tonearm_joint.motion_limits is not None,
        details="Expected separate vertical-axis platter and tonearm pivots.",
    )
    ctx.expect_origin_distance(
        platter,
        plinth,
        axes="xy",
        max_dist=0.001,
        name="platter_spindle_is_centered_on_plinth",
    )
    ctx.expect_contact(
        platter,
        plinth,
        elem_a="spindle_tip",
        elem_b="thrust_pad",
        name="platter_spindle_has_supported_thrust_contact",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        min_gap=0.030,
        max_gap=0.040,
        positive_elem="platter_disc",
        negative_elem="rain_cap",
        name="platter_disc_clears_weather_cap",
    )
    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="pivot_collar",
        elem_b="bearing_cap",
        name="tonearm_stage_has_supported_pivot_contact",
    )

    with ctx.pose({tonearm_joint: -0.30}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            min_overlap=0.012,
            elem_a="headshell",
            elem_b="platter_disc",
            name="tonearm_headshell_can_reach_record_area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            min_gap=0.0005,
            max_gap=0.004,
            positive_elem="cartridge",
            negative_elem="platter_mat",
            name="cartridge_hovers_just_above_platter_surface",
        )
        play_headshell = ctx.part_element_world_aabb(tonearm, elem="headshell")

    with ctx.pose({tonearm_joint: 0.60}):
        ctx.expect_gap(
            platter,
            tonearm,
            axis="y",
            min_gap=0.020,
            positive_elem="platter_disc",
            negative_elem="headshell",
            name="tonearm_parks_clear_of_platter",
        )
        park_headshell = ctx.part_element_world_aabb(tonearm, elem="headshell")

    if play_headshell is not None and park_headshell is not None:
        play_center = tuple(
            (play_headshell[0][i] + play_headshell[1][i]) / 2.0 for i in range(3)
        )
        park_center = tuple(
            (park_headshell[0][i] + park_headshell[1][i]) / 2.0 for i in range(3)
        )
        ctx.check(
            "tonearm_sweeps_between_play_and_park",
            park_center[1] < play_center[1] - 0.12 and park_center[0] > play_center[0] + 0.02,
            details=(
                f"Expected parked headshell to move aft and outward from play position; "
                f"play_center={play_center}, park_center={park_center}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
