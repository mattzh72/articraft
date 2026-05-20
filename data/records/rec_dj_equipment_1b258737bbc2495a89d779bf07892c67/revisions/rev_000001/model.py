from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_turntable")

    plinth_black = model.material("plinth_black", rgba=(0.11, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    platter_silver = model.material("platter_silver", rgba=(0.74, 0.75, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    label_red = model.material("label_red", rgba=(0.72, 0.17, 0.14, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.85, 0.49, 0.14, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.46, 0.355, 0.068)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=plinth_black,
        name="deck",
    )
    plinth.visual(
        Box((0.432, 0.327, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_black,
        name="underside_skirt",
    )
    plinth.visual(
        Cylinder(radius=0.031, length=0.010),
        origin=Origin(xyz=(-0.050, 0.0, 0.073)),
        material=satin_black,
        name="bearing_collar",
    )
    plinth.visual(
        Cylinder(radius=0.029, length=0.024),
        origin=Origin(xyz=(0.155, 0.112, 0.080)),
        material=satin_black,
        name="tonearm_mount",
    )
    plinth.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.155, 0.112, 0.097)),
        material=satin_black,
        name="tonearm_mount_top",
    )
    plinth.visual(
        Box((0.040, 0.016, 0.016)),
        origin=Origin(xyz=(0.132, -0.1695, 0.060)),
        material=satin_black,
        name="cue_mount",
    )
    plinth.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(-0.181, -0.124, 0.069)),
        material=aluminum,
        name="start_stop_button",
    )
    plinth.visual(
        Cylinder(radius=0.0035, length=0.0025),
        origin=Origin(xyz=(-0.181, -0.097, 0.06925)),
        material=accent_orange,
        name="status_led",
    )
    plinth.visual(
        Box((0.026, 0.128, 0.003)),
        origin=Origin(xyz=(0.188, 0.000, 0.0695)),
        material=aluminum,
        name="pitch_slider_track",
    )
    plinth.visual(
        Box((0.006, 0.110, 0.004)),
        origin=Origin(xyz=(0.188, 0.000, 0.070)),
        material=satin_black,
        name="pitch_slider_slot",
    )
    plinth.visual(
        Box((0.018, 0.016, 0.010)),
        origin=Origin(xyz=(0.188, -0.030, 0.073)),
        material=aluminum,
        name="pitch_slider_cap",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.355, 0.102)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.031, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=platter_silver,
        name="hub_sleeve",
    )
    platter.visual(
        Cylinder(radius=0.152, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=platter_silver,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.146, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        material=rubber,
        name="slipmat",
    )
    platter.visual(
        Cylinder(radius=0.022, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0218)),
        material=label_red,
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0286)),
        material=platter_silver,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.152, length=0.022),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    tonearm_base = model.part("tonearm_base")
    tonearm_base.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_black,
        name="pivot_cap",
    )
    tonearm_base.visual(
        Cylinder(radius=0.017, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=satin_black,
        name="yaw_housing",
    )
    tonearm_base.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.032)),
        material=satin_black,
        name="gimbal_web_left",
    )
    tonearm_base.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.032)),
        material=satin_black,
        name="gimbal_web_right",
    )
    tonearm_base.visual(
        Box((0.016, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.050)),
        material=satin_black,
        name="gimbal_cheek_left",
    )
    tonearm_base.visual(
        Box((0.016, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.050)),
        material=satin_black,
        name="gimbal_cheek_right",
    )
    tonearm_base.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.065)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
    )

    arm_tube_mesh = _save_mesh(
        "tonearm_tube",
        tube_from_spline_points(
            [
                (-0.012, 0.000, 0.000),
                (-0.052, 0.010, 0.006),
                (-0.116, 0.006, 0.005),
                (-0.185, -0.004, -0.001),
                (-0.230, -0.013, -0.010),
            ],
            radius=0.0055,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="gimbal_trunnion",
    )
    tonearm.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="pivot_socket",
    )
    tonearm.visual(
        arm_tube_mesh,
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.030, 0.016, 0.004)),
        origin=Origin(xyz=(-0.242, -0.013, -0.012), rpy=(0.16, 0.0, 0.0)),
        material=aluminum,
        name="headshell",
    )
    tonearm.visual(
        Box((0.014, 0.018, 0.010)),
        origin=Origin(xyz=(-0.247, -0.013, -0.020)),
        material=satin_black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0015, length=0.008),
        origin=Origin(xyz=(-0.252, -0.013, -0.028)),
        material=accent_orange,
        name="stylus",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="rear_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(0.056, 0.0, 0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.320, 0.060, 0.070)),
        mass=0.38,
        origin=Origin(xyz=(-0.095, 0.0, -0.001)),
    )

    cue_lever = model.part("cue_lever")
    cue_lever.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lever_barrel",
    )
    cue_lever.visual(
        Box((0.010, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.022, 0.006)),
        material=accent_orange,
        name="lever_stalk",
    )
    cue_lever.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.048, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_orange,
        name="lever_knob",
    )
    cue_lever.inertial = Inertial.from_geometry(
        Box((0.032, 0.070, 0.024)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.018, 0.008)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.050, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm_base,
        origin=Origin(xyz=(0.155, 0.112, 0.102)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.2,
            lower=-1.0,
            upper=0.65,
        ),
    )
    model.articulation(
        "tonearm_height",
        ArticulationType.REVOLUTE,
        parent=tonearm_base,
        child=tonearm,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=1.8,
            lower=-0.15,
            upper=0.35,
        ),
    )
    model.articulation(
        "cue_lever_hinge",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=cue_lever,
        origin=Origin(xyz=(0.132, -0.1775, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm_base = object_model.get_part("tonearm_base")
    tonearm = object_model.get_part("tonearm")
    cue_lever = object_model.get_part("cue_lever")

    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")
    tonearm_height = object_model.get_articulation("tonearm_height")
    cue_hinge = object_model.get_articulation("cue_lever_hinge")

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
        "platter spin uses vertical continuous axis",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS
        and platter_spin.axis == (0.0, 0.0, 1.0),
        details=str(
            {
                "type": getattr(platter_spin.articulation_type, "value", platter_spin.articulation_type),
                "axis": platter_spin.axis,
            }
        ),
    )
    ctx.check(
        "tonearm swing uses vertical pivot axis",
        tonearm_swing.axis == (0.0, 0.0, 1.0),
        details=str({"axis": tonearm_swing.axis}),
    )
    ctx.check(
        "tonearm height uses horizontal pivot axis",
        tonearm_height.axis == (0.0, 1.0, 0.0),
        details=str({"axis": tonearm_height.axis}),
    )
    ctx.check(
        "cue lever hinge uses horizontal front-edge axis",
        cue_hinge.axis == (1.0, 0.0, 0.0),
        details=str({"axis": cue_hinge.axis}),
    )

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="hub_sleeve",
        elem_b="bearing_collar",
        name="platter hub is supported by the central bearing collar",
    )
    ctx.expect_contact(
        tonearm_base,
        plinth,
        elem_a="pivot_cap",
        elem_b="tonearm_mount_top",
        name="tonearm base is seated on the plinth mount",
    )
    ctx.expect_contact(
        tonearm,
        tonearm_base,
        elem_a="gimbal_trunnion",
        elem_b="gimbal_cheek_left",
        name="tonearm trunnion is supported by the left gimbal cheek",
    )
    ctx.expect_contact(
        cue_lever,
        plinth,
        elem_a="lever_barrel",
        elem_b="cue_mount",
        name="cue lever is hinged from the plinth front bracket",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        min_overlap=0.28,
        name="platter footprint sits well within the plinth deck",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus",
        negative_elem="slipmat",
        min_gap=0.006,
        max_gap=0.030,
        name="stylus hovers above the record surface in the rest pose",
    )

    rest_stylus = ctx.part_element_world_aabb(tonearm, elem="stylus")
    with ctx.pose({tonearm_height: 0.25}):
        raised_stylus = ctx.part_element_world_aabb(tonearm, elem="stylus")
    ctx.check(
        "tonearm height joint raises the stylus",
        rest_stylus is not None
        and raised_stylus is not None
        and raised_stylus[0][2] > rest_stylus[0][2] + 0.02,
        details=f"rest={rest_stylus}, raised={raised_stylus}",
    )

    with ctx.pose({tonearm_swing: 0.40}):
        swung_stylus = ctx.part_element_world_aabb(tonearm, elem="stylus")
    ctx.check(
        "tonearm swing moves the stylus laterally across the deck",
        rest_stylus is not None
        and swung_stylus is not None
        and abs(swung_stylus[0][1] - rest_stylus[0][1]) > 0.04,
        details=f"rest={rest_stylus}, swung={swung_stylus}",
    )

    rest_knob = ctx.part_element_world_aabb(cue_lever, elem="lever_knob")
    with ctx.pose({cue_hinge: 0.55}):
        raised_knob = ctx.part_element_world_aabb(cue_lever, elem="lever_knob")
    ctx.check(
        "cue lever lifts upward when actuated",
        rest_knob is not None
        and raised_knob is not None
        and raised_knob[0][2] > rest_knob[0][2] + 0.015,
        details=f"rest={rest_knob}, raised={raised_knob}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
