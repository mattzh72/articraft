from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _build_jug_shell_mesh():
    outer_profile = [
        (0.040, 0.000),
        (0.044, 0.008),
        (0.050, 0.028),
        (0.058, 0.072),
        (0.066, 0.132),
        (0.074, 0.184),
        (0.078, 0.198),
    ]
    inner_profile = [
        (0.014, 0.010),
        (0.038, 0.016),
        (0.044, 0.032),
        (0.052, 0.074),
        (0.060, 0.132),
        (0.068, 0.186),
        (0.071, 0.194),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("smoothie_blender_jug_shell.obj"))


def _build_handle_mesh():
    geom = tube_from_spline_points(
        [
            (-0.056, 0.000, 0.044),
            (-0.086, 0.000, 0.068),
            (-0.097, 0.000, 0.118),
            (-0.088, 0.000, 0.168),
            (-0.072, 0.000, 0.188),
        ],
        radius=0.007,
        samples_per_segment=18,
        radial_segments=20,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("smoothie_blender_handle.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smoothie_blender", assets=ASSETS)

    graphite = model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.11, 0.12, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.82, 0.90, 0.96, 0.28))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    satin = model.material("satin", rgba=(0.58, 0.61, 0.66, 1.0))
    accent = model.material("teal_accent", rgba=(0.24, 0.62, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.076, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        name="foot_ring",
        material=graphite,
    )
    base.visual(
        Cylinder(radius=0.070, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        name="motor_housing",
        material=graphite,
    )
    base.visual(
        Cylinder(radius=0.056, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        name="shoulder",
        material=graphite,
    )
    base.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        name="socket",
        material=charcoal,
    )
    base.visual(
        Box((0.016, 0.060, 0.042)),
        origin=Origin(xyz=(0.061, 0.0, 0.051)),
        name="control_panel",
        material=satin,
    )
    base.visual(
        Box((0.008, 0.018, 0.006)),
        origin=Origin(xyz=(0.069, 0.0, 0.071)),
        name="pulse_button",
        material=accent,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.152, 0.152, 0.110)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    control_dial = model.part("control_dial")
    control_dial.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="dial_stem",
        material=satin,
    )
    control_dial.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="knob",
        material=accent,
    )
    control_dial.visual(
        Box((0.003, 0.010, 0.005)),
        origin=Origin(xyz=(0.018, 0.0, 0.010)),
        name="indicator",
        material=charcoal,
    )
    control_dial.inertial = Inertial.from_geometry(
        Box((0.022, 0.026, 0.026)),
        mass=0.08,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
    )

    jug = model.part("jug")
    jug.visual(_build_jug_shell_mesh(), name="shell", material=smoked_clear)
    jug.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="bottom_collar",
        material=charcoal,
    )
    jug.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        name="drive_coupler",
        material=satin,
    )
    jug.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        name="blade_bearing",
        material=satin,
    )
    jug.visual(_build_handle_mesh(), name="handle", material=graphite)
    jug.visual(
        Box((0.018, 0.034, 0.010)),
        origin=Origin(xyz=(0.071, 0.0, 0.189), rpy=(0.0, math.radians(-18.0), 0.0)),
        name="spout",
        material=smoked_clear,
    )
    jug.inertial = Inertial.from_geometry(
        Box((0.196, 0.156, 0.198)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
    )

    blades = model.part("blade_unit")
    blades.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        name="hub_flange",
        material=satin,
    )
    blades.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        name="shaft",
        material=satin,
    )
    blades.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        name="upper_nut",
        material=satin,
    )
    blades.visual(
        Box((0.052, 0.012, 0.0024)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.010),
            rpy=(0.0, math.radians(14.0), math.radians(25.0)),
        ),
        name="lower_blade_a",
        material=steel,
    )
    blades.visual(
        Box((0.052, 0.012, 0.0024)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.010),
            rpy=(0.0, math.radians(-12.0), math.radians(115.0)),
        ),
        name="lower_blade_b",
        material=steel,
    )
    blades.visual(
        Box((0.046, 0.012, 0.0024)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.022),
            rpy=(0.0, math.radians(-14.0), math.radians(115.0)),
        ),
        name="upper_blade_a",
        material=steel,
    )
    blades.visual(
        Box((0.046, 0.012, 0.0024)),
        origin=Origin(
            xyz=(0.0, 0.0, 0.022),
            rpy=(0.0, math.radians(12.0), math.radians(25.0)),
        ),
        name="upper_blade_b",
        material=steel,
    )
    blades.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.030)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.080, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        name="top_cap",
        material=charcoal,
    )
    lid.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        name="fill_cap_detail",
        material=accent,
    )
    lid.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        name="seal_skirt",
        material=graphite,
    )
    lid.visual(
        Box((0.018, 0.012, 0.004)),
        origin=Origin(xyz=(0.052, 0.0, -0.002)),
        name="lug_left",
        material=graphite,
    )
    lid.visual(
        Box((0.018, 0.012, 0.004)),
        origin=Origin(xyz=(-0.052, 0.0, -0.002)),
        name="lug_right",
        material=graphite,
    )
    lid.visual(
        Box((0.020, 0.022, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, 0.011)),
        name="alignment_tab",
        material=graphite,
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.160, 0.160, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "base_to_control_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=control_dial,
        origin=Origin(xyz=(0.069, 0.0, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0),
    )
    model.articulation(
        "base_to_jug",
        ArticulationType.FIXED,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )
    model.articulation(
        "jug_to_blades",
        ArticulationType.CONTINUOUS,
        parent=jug,
        child=blades,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "jug_to_lid",
        ArticulationType.REVOLUTE,
        parent=jug,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=0.52),
    )

    return model


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    control_dial = object_model.get_part("control_dial")
    jug = object_model.get_part("jug")
    blades = object_model.get_part("blade_unit")
    lid = object_model.get_part("lid")

    dial_spin = object_model.get_articulation("base_to_control_dial")
    blade_spin = object_model.get_articulation("jug_to_blades")
    lid_twist = object_model.get_articulation("jug_to_lid")

    base_socket = base.get_visual("socket")
    jug_collar = jug.get_visual("bottom_collar")
    blade_bearing = jug.get_visual("blade_bearing")
    jug_shell = jug.get_visual("shell")
    hub = blades.get_visual("hub_flange")
    lower_blade_a = blades.get_visual("lower_blade_a")
    upper_blade_a = blades.get_visual("upper_blade_a")
    lid_cap = lid.get_visual("top_cap")
    seal_skirt = lid.get_visual("seal_skirt")
    dial_indicator = control_dial.get_visual("indicator")
    alignment_tab = lid.get_visual("alignment_tab")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    base_aabb = ctx.part_world_aabb(base)
    jug_aabb = ctx.part_world_aabb(jug)
    lid_aabb = ctx.part_world_aabb(lid)
    assert base_aabb is not None
    assert jug_aabb is not None
    assert lid_aabb is not None

    base_size = _aabb_size(base_aabb)
    jug_size = _aabb_size(jug_aabb)
    lid_size = _aabb_size(lid_aabb)
    ctx.check(
        "compact_base_proportions",
        0.095 <= base_size[2] <= 0.125 and max(base_size[0], base_size[1]) <= 0.160,
        f"base size={base_size}",
    )
    ctx.check(
        "wide_mouth_jug_proportions",
        0.190 <= jug_size[2] <= 0.240 and max(lid_size[0], lid_size[1]) >= 0.150,
        f"jug size={jug_size}, lid size={lid_size}",
    )

    ctx.expect_contact(base, jug, elem_a=base_socket, elem_b=jug_collar)
    ctx.expect_gap(
        jug,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=jug_collar,
        negative_elem=base_socket,
        name="jug_seated_on_base_socket",
    )
    ctx.expect_origin_distance(jug, base, axes="xy", max_dist=0.003)
    ctx.expect_overlap(jug, base, axes="xy", min_overlap=0.074)
    ctx.expect_contact(control_dial, base)

    ctx.expect_contact(blades, jug, elem_a=hub, elem_b=blade_bearing)
    ctx.expect_origin_distance(blades, jug, axes="xy", max_dist=0.002)
    ctx.expect_gap(
        blades,
        jug,
        axis="z",
        min_gap=0.006,
        max_gap=0.013,
        positive_elem=lower_blade_a,
        negative_elem=blade_bearing,
        name="lower_blade_cluster_above_bearing",
    )
    ctx.expect_gap(
        blades,
        jug,
        axis="z",
        min_gap=0.016,
        max_gap=0.030,
        positive_elem=upper_blade_a,
        negative_elem=blade_bearing,
        name="upper_blade_cluster_above_bearing",
    )

    ctx.expect_gap(
        lid,
        jug,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lid_cap,
        negative_elem=jug_shell,
        name="lid_seated_on_jug",
    )
    ctx.expect_contact(lid, jug)
    ctx.expect_origin_distance(lid, jug, axes="xy", max_dist=0.002)
    ctx.expect_overlap(lid, jug, axes="xy", min_overlap=0.120)
    ctx.expect_within(lid, jug, axes="xy", inner_elem=seal_skirt, outer_elem=jug_shell)

    dial_indicator_rest = ctx.part_element_world_aabb(control_dial, elem=dial_indicator.name)
    blade_rest = ctx.part_element_world_aabb(blades, elem=lower_blade_a.name)
    lid_tab_rest = ctx.part_element_world_aabb(lid, elem=alignment_tab.name)
    assert dial_indicator_rest is not None
    assert blade_rest is not None
    assert lid_tab_rest is not None

    with ctx.pose({dial_spin: math.pi / 2.0}):
        dial_indicator_quarter = ctx.part_element_world_aabb(control_dial, elem=dial_indicator.name)
        assert dial_indicator_quarter is not None
        rest_center = _aabb_center(dial_indicator_rest)
        turn_center = _aabb_center(dial_indicator_quarter)
        ctx.check(
            "control_dial_rotates",
            abs(turn_center[1] - rest_center[1]) > 0.007 and abs(turn_center[2] - rest_center[2]) > 0.004,
            f"rest={rest_center}, turned={turn_center}",
        )
        ctx.expect_contact(control_dial, base)

    with ctx.pose({blade_spin: 2.1}):
        blade_spun = ctx.part_element_world_aabb(blades, elem=lower_blade_a.name)
        assert blade_spun is not None
        rest_span_y = blade_rest[1][1] - blade_rest[0][1]
        spun_span_y = blade_spun[1][1] - blade_spun[0][1]
        ctx.check(
            "blade_cluster_spins",
            abs(spun_span_y - rest_span_y) > 0.020,
            f"rest_y_span={rest_span_y}, spun_y_span={spun_span_y}",
        )
        ctx.expect_contact(blades, jug, elem_a=hub, elem_b=blade_bearing)
        ctx.expect_gap(
            blades,
            jug,
            axis="z",
            min_gap=0.006,
            max_gap=0.013,
            positive_elem=lower_blade_a,
            negative_elem=blade_bearing,
            name="lower_blade_cluster_above_bearing_spun",
        )

    lid_limits = lid_twist.motion_limits
    assert lid_limits is not None and lid_limits.upper is not None
    with ctx.pose({lid_twist: lid_limits.upper}):
        lid_tab_turned = ctx.part_element_world_aabb(lid, elem=alignment_tab.name)
        assert lid_tab_turned is not None
        rest_center = _aabb_center(lid_tab_rest)
        turned_center = _aabb_center(lid_tab_turned)
        ctx.check(
            "lid_twists_to_lock",
            math.dist(rest_center[:2], turned_center[:2]) > 0.018,
            f"rest={rest_center}, turned={turned_center}",
        )
        ctx.expect_contact(lid, jug)
        ctx.expect_gap(
            lid,
            jug,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=lid_cap,
            negative_elem=jug_shell,
            name="lid_stays_seated_while_twisting",
        )
        ctx.expect_origin_distance(lid, jug, axes="xy", max_dist=0.002)
        ctx.expect_overlap(lid, jug, axes="xy", min_overlap=0.120)
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_twist_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_twist_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
