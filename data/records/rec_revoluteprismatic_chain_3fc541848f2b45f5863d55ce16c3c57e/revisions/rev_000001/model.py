from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BACKPLATE_WIDTH = 0.16
BACKPLATE_HEIGHT = 0.26
BACKPLATE_THICKNESS = 0.012

HINGE_BRACKET_THICKNESS = 0.016
HINGE_BRACKET_LENGTH = 0.046
HINGE_BRACKET_HEIGHT = 0.088
HINGE_BRACKET_CENTER_X = -0.089
HINGE_CENTER_Y = 0.023

LEAF_THICKNESS = 0.016
LEAF_LENGTH = 0.055
LEAF_HEIGHT = 0.05
BARREL_RADIUS = 0.013
BARREL_LENGTH = 0.078

BEAM_WIDTH = 0.03
BEAM_HEIGHT = 0.026
BEAM_LENGTH = 0.25
BEAM_CENTER_Y = 0.155

GUIDE_WIDTH = 0.058
GUIDE_HEIGHT = 0.05
GUIDE_LENGTH = 0.06
GUIDE_CENTER_Y = 0.303
GUIDE_FRONT_Y = GUIDE_CENTER_Y + GUIDE_LENGTH / 2.0
GUIDE_RAIL_THICKNESS = 0.012
GUIDE_RAIL_Z = 0.019

RAM_FLANGE_RADIUS = 0.016
RAM_FLANGE_LENGTH = 0.008
RAM_ROD_RADIUS = 0.009
RAM_ROD_LENGTH = 0.09
RAM_PAD_WIDTH = 0.026
RAM_PAD_HEIGHT = 0.026
RAM_PAD_LENGTH = 0.01


def _make_backplate_shape() -> cq.Workplane:
    hole_x = BACKPLATE_WIDTH * 0.29
    hole_z = BACKPLATE_HEIGHT * 0.31
    mount_hole_d = 0.009

    plate = (
        cq.Workplane("XY")
        .box(BACKPLATE_WIDTH, BACKPLATE_THICKNESS, BACKPLATE_HEIGHT)
        .edges("|Y")
        .fillet(0.012)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-hole_x, -hole_z),
                (hole_x, -hole_z),
                (-hole_x, hole_z),
                (hole_x, hole_z),
            ]
        )
        .hole(mount_hole_d)
    )

    hinge_bracket = cq.Workplane("XY").box(
        HINGE_BRACKET_THICKNESS, HINGE_BRACKET_LENGTH, HINGE_BRACKET_HEIGHT
    ).translate((HINGE_BRACKET_CENTER_X, HINGE_CENTER_Y, 0.0))
    gusset = cq.Workplane("XY").box(
        0.022, 0.024, 0.07
    ).translate((HINGE_BRACKET_CENTER_X + 0.006, 0.011, 0.0))
    bracket_cap = (
        cq.Workplane("YZ")
        .circle(0.016)
        .extrude(0.008)
        .translate((HINGE_BRACKET_CENTER_X - HINGE_BRACKET_THICKNESS / 2.0, HINGE_CENTER_Y, 0.0))
    )

    return plate.union(hinge_bracket).union(gusset).union(bracket_cap)


def _make_main_link_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .circle(BARREL_RADIUS)
        .extrude(BARREL_LENGTH)
        .translate((0.0, 0.0, -BARREL_LENGTH / 2.0))
    )

    hinge_leaf = cq.Workplane("XY").box(LEAF_THICKNESS, LEAF_LENGTH, LEAF_HEIGHT).translate(
        (-0.018, 0.021, 0.0)
    )
    proximal_web = cq.Workplane("XY").box(0.034, 0.06, 0.042).translate(
        (-0.017, 0.042, 0.0)
    )
    beam = cq.Workplane("XY").box(BEAM_WIDTH, BEAM_LENGTH, BEAM_HEIGHT).translate(
        (-0.015, BEAM_CENTER_Y, 0.0)
    )
    rear_bridge = cq.Workplane("XY").box(GUIDE_WIDTH, 0.03, 0.036).translate(
        (-GUIDE_WIDTH / 2.0, GUIDE_CENTER_Y - 0.02, 0.0)
    )
    top_rail = cq.Workplane("XY").box(
        GUIDE_WIDTH, GUIDE_LENGTH, GUIDE_RAIL_THICKNESS
    ).translate((-GUIDE_WIDTH / 2.0, GUIDE_CENTER_Y, GUIDE_RAIL_Z))
    bottom_rail = cq.Workplane("XY").box(
        GUIDE_WIDTH, GUIDE_LENGTH, GUIDE_RAIL_THICKNESS
    ).translate((-GUIDE_WIDTH / 2.0, GUIDE_CENTER_Y, -GUIDE_RAIL_Z))

    return (
        barrel.union(hinge_leaf)
        .union(proximal_web)
        .union(beam)
        .union(rear_bridge)
        .union(top_rail)
        .union(bottom_rail)
    )


def _make_ram_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .circle(RAM_FLANGE_RADIUS)
        .extrude(RAM_FLANGE_LENGTH)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )
    rod = (
        cq.Workplane("XY")
        .circle(RAM_ROD_RADIUS)
        .extrude(RAM_ROD_LENGTH)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, RAM_FLANGE_LENGTH - 0.002, 0.0))
    )
    pad = cq.Workplane("XY").box(RAM_PAD_WIDTH, RAM_PAD_LENGTH, RAM_PAD_HEIGHT).translate(
        (
            0.0,
            RAM_FLANGE_LENGTH + RAM_ROD_LENGTH + RAM_PAD_LENGTH / 2.0 - 0.004,
            0.0,
        )
    )

    return flange.union(rod).union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_swing_slide_fixture")

    model.material("backplate_finish", color=(0.22, 0.24, 0.27))
    model.material("arm_finish", color=(0.62, 0.66, 0.7))
    model.material("ram_finish", color=(0.79, 0.81, 0.83))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_make_backplate_shape(), "backplate"),
        origin=Origin(),
        material="backplate_finish",
        name="backplate_body",
    )

    main_link = model.part("main_link")
    main_link.visual(
        mesh_from_cadquery(_make_main_link_shape(), "main_link"),
        origin=Origin(),
        material="arm_finish",
        name="main_link_body",
    )

    ram = model.part("ram")
    ram.visual(
        mesh_from_cadquery(_make_ram_shape(), "ram"),
        origin=Origin(),
        material="ram_finish",
        name="ram_body",
    )

    model.articulation(
        "backplate_to_main_link",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=main_link,
        origin=Origin(
            xyz=(
                HINGE_BRACKET_CENTER_X - HINGE_BRACKET_THICKNESS / 2.0 - BARREL_RADIUS,
                HINGE_CENTER_Y,
                0.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-1.05,
            upper=1.05,
        ),
    )

    model.articulation(
        "main_link_to_ram",
        ArticulationType.PRISMATIC,
        parent=main_link,
        child=ram,
        origin=Origin(xyz=(-GUIDE_WIDTH / 2.0, GUIDE_FRONT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.15,
            lower=0.0,
            upper=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    main_link = object_model.get_part("main_link")
    ram = object_model.get_part("ram")
    swing = object_model.get_articulation("backplate_to_main_link")
    slide = object_model.get_articulation("main_link_to_ram")

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
        "fixture_parts_exist",
        all(part is not None for part in (backplate, main_link, ram)),
        "Expected backplate, main_link, and ram parts to exist.",
    )
    ctx.check(
        "swing_joint_is_vertical_revolute",
        swing.articulation_type == ArticulationType.REVOLUTE
        and abs(swing.axis[0]) < 1e-9
        and abs(swing.axis[1]) < 1e-9
        and abs(swing.axis[2] - 1.0) < 1e-9
        and swing.motion_limits is not None
        and swing.motion_limits.lower is not None
        and swing.motion_limits.upper is not None
        and swing.motion_limits.lower < 0.0 < swing.motion_limits.upper,
        f"Unexpected swing joint configuration: type={swing.articulation_type}, axis={swing.axis}, limits={swing.motion_limits}",
    )
    ctx.check(
        "ram_joint_is_distal_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC
        and abs(slide.axis[0]) < 1e-9
        and abs(slide.axis[1] - 1.0) < 1e-9
        and abs(slide.axis[2]) < 1e-9
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.08
        and slide.origin.xyz[1] > 0.3,
        f"Unexpected ram joint configuration: type={slide.articulation_type}, axis={slide.axis}, origin={slide.origin.xyz}, limits={slide.motion_limits}",
    )

    with ctx.pose({swing: 0.0, slide: 0.0}):
        ctx.expect_contact(
            main_link,
            backplate,
            contact_tol=0.001,
            name="hinge_barrel_seated_in_backplate_clevis",
        )
        ctx.expect_contact(
            ram,
            main_link,
            contact_tol=0.001,
            name="ram_flange_seated_on_distal_guide",
        )
        ctx.expect_overlap(
            ram,
            main_link,
            axes="xz",
            min_overlap=0.024,
            name="ram_stays_aligned_with_distal_guide",
        )

    with ctx.pose({swing: 0.0, slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            ram,
            main_link,
            axis="y",
            min_gap=0.079,
            max_gap=0.081,
            name="ram_extends_forward_from_the_guide",
        )
        ctx.expect_overlap(
            ram,
            main_link,
            axes="xz",
            min_overlap=0.024,
            name="extended_ram_remains_coaxial_with_guide",
        )

    with ctx.pose({swing: 0.0, slide: 0.0}):
        ram_retracted = ctx.part_world_position(ram)
    with ctx.pose({swing: 0.0, slide: slide.motion_limits.upper}):
        ram_extended = ctx.part_world_position(ram)
    ctx.check(
        "ram_translates_outward_along_local_axis",
        ram_retracted is not None
        and ram_extended is not None
        and ram_extended[1] > ram_retracted[1] + 0.075
        and abs(ram_extended[0] - ram_retracted[0]) < 1e-6
        and abs(ram_extended[2] - ram_retracted[2]) < 1e-6,
        f"Expected ram extension along +Y in the neutral swing pose, got retracted={ram_retracted}, extended={ram_extended}",
    )

    with ctx.pose({swing: 0.0, slide: 0.0}):
        ram_neutral = ctx.part_world_position(ram)
    with ctx.pose({swing: 0.85, slide: 0.0}):
        ram_swung = ctx.part_world_position(ram)
    ctx.check(
        "main_link_swings_sideways_about_vertical_hinge",
        ram_neutral is not None
        and ram_swung is not None
        and ram_swung[0] < ram_neutral[0] - 0.2
        and ram_swung[1] < ram_neutral[1] - 0.08,
        f"Expected positive swing to move the distal end sideways around the wall plate, got neutral={ram_neutral}, swung={ram_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
