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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scratch_turntable")

    plinth_black = model.material("plinth_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    label_blue = model.material("label_blue", rgba=(0.16, 0.22, 0.34, 1.0))
    counterweight_dark = model.material("counterweight_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.88, 0.47, 0.12, 1.0))

    plinth = model.part("plinth")
    plinth_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.45, 0.355, 0.018, corner_segments=10),
        0.05,
    )
    plinth.visual(
        _mesh("turntable_plinth_shell", plinth_shell),
        material=plinth_black,
        name="plinth_shell",
    )
    plinth.visual(
        Box((0.41, 0.315, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=counterweight_dark,
        name="sub_base",
    )
    plinth.visual(
        Cylinder(radius=0.035, length=0.004),
        origin=Origin(xyz=(-0.06, 0.01, 0.052)),
        material=satin_aluminum,
        name="motor_hub",
    )
    plinth.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.155, 0.082, 0.053)),
        material=satin_aluminum,
        name="armboard",
    )
    plinth.visual(
        Box((0.078, 0.05, 0.001)),
        origin=Origin(xyz=(0.168, -0.115, 0.0505)),
        material=satin_aluminum,
        name="speed_plate",
    )
    plinth.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(xyz=(0.112, -0.05, 0.061)),
        material=steel,
        name="arm_rest_post",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.45, 0.355, 0.058)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    platter = model.part("platter")
    platter_profile = [
        (0.0, 0.0),
        (0.05, 0.0),
        (0.11, 0.0025),
        (0.152, 0.0050),
        (0.162, 0.010),
        (0.166, 0.014),
        (0.166, 0.030),
        (0.152, 0.0315),
        (0.10, 0.0315),
        (0.0, 0.0305),
    ]
    platter.visual(
        _mesh("turntable_platter_body", LatheGeometry(platter_profile, segments=96)),
        material=satin_aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.148, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0330)),
        material=dark_rubber,
        name="slip_mat",
    )
    platter.visual(
        Cylinder(radius=0.055, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0351)),
        material=label_blue,
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0380)),
        material=steel,
        name="spindle_tip",
    )
    platter.visual(
        Box((0.010, 0.004, 0.0015)),
        origin=Origin(xyz=(0.145, 0.0, 0.0332)),
        material=steel,
        name="strobe_mark",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.166, length=0.036),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_aluminum,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=satin_aluminum,
        name="pivot_stem",
    )
    tonearm.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="gimbal_tube",
    )
    tonearm.visual(
        _mesh(
            "turntable_tonearm_tube",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.034),
                    (-0.016, -0.026, 0.036),
                    (-0.052, -0.072, 0.039),
                    (-0.024, -0.118, 0.041),
                    (-0.056, -0.166, 0.042),
                ],
                radius=0.0048,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=satin_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        _mesh(
            "turntable_rear_stub",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.034),
                    (0.010, 0.014, 0.034),
                    (0.028, 0.033, 0.034),
                ],
                radius=0.0043,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=satin_aluminum,
        name="rear_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(0.030, 0.037, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=counterweight_dark,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.018, 0.040, 0.004)),
        origin=Origin(xyz=(-0.057, -0.181, 0.041), rpy=(0.0, 0.0, 0.25)),
        material=satin_aluminum,
        name="headshell",
    )
    tonearm.visual(
        Box((0.010, 0.016, 0.008)),
        origin=Origin(xyz=(-0.059, -0.196, 0.038)),
        material=counterweight_dark,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0010, length=0.002),
        origin=Origin(xyz=(-0.060, -0.201, 0.0345)),
        material=counterweight_dark,
        name="stylus",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.11, 0.25, 0.06)),
        mass=0.35,
        origin=Origin(xyz=(-0.012, -0.068, 0.032)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_aluminum,
        name="selector_body",
    )
    speed_selector.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=counterweight_dark,
        name="selector_cap",
    )
    speed_selector.visual(
        Box((0.004, 0.012, 0.0015)),
        origin=Origin(xyz=(0.009, 0.0, 0.0147)),
        material=accent_orange,
        name="selector_mark",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.016),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.06, 0.01, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "tonearm_sweep",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.155, 0.082, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=2.0,
            lower=-1.05,
            upper=0.18,
        ),
    )
    model.articulation(
        "speed_select",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=speed_selector,
        origin=Origin(xyz=(0.168, -0.115, 0.051)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=1.5,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    speed_selector = object_model.get_part("speed_selector")

    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_sweep = object_model.get_articulation("tonearm_sweep")
    speed_select = object_model.get_articulation("speed_select")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(platter, plinth, elem_b="motor_hub", name="platter rests on motor hub")
    ctx.expect_contact(tonearm, plinth, elem_a="pivot_collar", elem_b="armboard", name="tonearm is mounted on armboard")
    ctx.expect_contact(
        speed_selector,
        plinth,
        elem_a="selector_body",
        elem_b="speed_plate",
        name="speed selector is mounted on control plate",
    )

    ctx.check(
        "platter articulation is continuous vertical spin",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(platter_spin.axis) == (0.0, 0.0, 1.0),
        details=f"got type={platter_spin.articulation_type} axis={platter_spin.axis}",
    )
    ctx.check(
        "tonearm articulation sweeps around vertical pivot",
        tonearm_sweep.articulation_type == ArticulationType.REVOLUTE
        and tuple(tonearm_sweep.axis) == (0.0, 0.0, 1.0)
        and tonearm_sweep.motion_limits is not None
        and tonearm_sweep.motion_limits.lower is not None
        and tonearm_sweep.motion_limits.upper is not None
        and tonearm_sweep.motion_limits.lower <= -0.9
        and tonearm_sweep.motion_limits.upper >= 0.1,
        details=f"got axis={tonearm_sweep.axis} limits={tonearm_sweep.motion_limits}",
    )
    ctx.check(
        "speed selector articulation has two-speed range",
        speed_select.articulation_type == ArticulationType.REVOLUTE
        and tuple(speed_select.axis) == (0.0, 0.0, 1.0)
        and speed_select.motion_limits is not None
        and speed_select.motion_limits.lower is not None
        and speed_select.motion_limits.upper is not None
        and speed_select.motion_limits.lower <= -0.3
        and speed_select.motion_limits.upper >= 0.3,
        details=f"got axis={speed_select.axis} limits={speed_select.motion_limits}",
    )

    strobe_rest = _aabb_center(ctx.part_element_world_aabb(platter, elem="strobe_mark"))
    with ctx.pose({platter_spin: math.pi / 2.0}):
        strobe_quarter_turn = _aabb_center(ctx.part_element_world_aabb(platter, elem="strobe_mark"))
        ctx.check(
            "platter marker advances around the spindle",
            strobe_rest is not None
            and strobe_quarter_turn is not None
            and strobe_quarter_turn[0] < strobe_rest[0] - 0.10
            and strobe_quarter_turn[1] > strobe_rest[1] + 0.10,
            details=f"rest={strobe_rest} quarter_turn={strobe_quarter_turn}",
        )

    headshell_parked = _aabb_center(ctx.part_element_world_aabb(tonearm, elem="headshell"))
    with ctx.pose({tonearm_sweep: -0.92}):
        headshell_play = _aabb_center(ctx.part_element_world_aabb(tonearm, elem="headshell"))
        ctx.check(
            "tonearm swings inward over the platter",
            headshell_parked is not None
            and headshell_play is not None
            and headshell_play[0] < headshell_parked[0] - 0.08
            and headshell_play[1] > headshell_parked[1] + 0.07,
            details=f"parked={headshell_parked} play={headshell_play}",
        )
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="slip_mat",
            min_overlap=0.012,
            name="headshell can reach the record area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="stylus",
            negative_elem="slip_mat",
            min_gap=0.0005,
            max_gap=0.004,
            name="stylus hovers just above the platter mat",
        )

    selector_rest = _aabb_center(ctx.part_element_world_aabb(speed_selector, elem="selector_mark"))
    with ctx.pose({speed_select: 0.30}):
        selector_turned = _aabb_center(ctx.part_element_world_aabb(speed_selector, elem="selector_mark"))
        ctx.check(
            "speed selector knob turns",
            selector_rest is not None
            and selector_turned is not None
            and selector_turned[1] > selector_rest[1] + 0.002,
            details=f"rest={selector_rest} turned={selector_turned}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
