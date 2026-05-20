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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_station_pump")

    base_mat = model.material("base_dark", color=(0.20, 0.20, 0.22, 1.0))
    steel_mat = model.material("steel", color=(0.72, 0.74, 0.76, 1.0))
    gauge_mat = model.material("gauge_dark", color=(0.13, 0.14, 0.15, 1.0))
    grip_mat = model.material("grip_black", color=(0.10, 0.10, 0.10, 1.0))
    knob_mat = model.material("selector_red", color=(0.70, 0.12, 0.08, 1.0))

    base_height = 0.050
    base_pedestal_height = 0.032
    barrel_height = 0.720
    barrel_outer_r = 0.040
    barrel_inner_r = 0.033
    rod_radius = 0.010
    rod_hidden = 0.470
    rod_exposed = 0.235
    plunger_travel = 0.260
    gauge_mount_z = 0.430

    base_shape = (
        cq.Workplane("XY")
        .box(0.340, 0.220, base_height, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.055)
        .extrude(base_pedestal_height)
    )

    barrel_body = cq.Workplane("XY").cylinder(barrel_height, barrel_outer_r, centered=(True, True, False))
    barrel_bore = cq.Workplane("XY").cylinder(
        barrel_height - 0.010,
        barrel_inner_r,
        centered=(True, True, False),
    ).translate((0.0, 0.0, 0.010))
    top_gland = (
        cq.Workplane("XY")
        .cylinder(0.032, barrel_outer_r + 0.007, centered=(True, True, False))
        .translate((0.0, 0.0, barrel_height - 0.016))
    )
    top_gland_hole = (
        cq.Workplane("XY")
        .cylinder(0.038, rod_radius + 0.0025, centered=(True, True, False))
        .translate((0.0, 0.0, barrel_height - 0.019))
    )
    barrel_shape = (
        barrel_body.cut(barrel_bore)
        .union(top_gland)
        .cut(top_gland_hole)
    )

    gauge_case_depth = 0.046
    gauge_case_back_y = barrel_outer_r + 0.039
    gauge_case_front_y = gauge_case_back_y + gauge_case_depth
    clamp_outer_r = barrel_outer_r + 0.012
    clamp_inner_r = barrel_outer_r - 0.001
    clamp_band = (
        cq.Workplane("XY")
        .circle(clamp_outer_r)
        .circle(clamp_inner_r)
        .extrude(0.058, both=True)
        .cut(
            cq.Workplane("XY")
            .box(0.180, 0.094, 0.180, centered=True)
            .translate((0.0, -0.047, 0.0))
        )
    )
    gauge_arm = (
        cq.Workplane("XY")
        .box(0.044, 0.036, 0.120, centered=True)
        .translate((0.0, clamp_outer_r + 0.018, 0.0))
    )
    gauge_case = (
        cq.Workplane("XY")
        .box(0.110, gauge_case_depth, 0.150, centered=True)
        .edges("|Z")
        .fillet(0.010)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .circle(0.038)
        .cutBlind(-0.004)
        .translate((0.0, gauge_case_back_y + gauge_case_depth / 2.0, 0.0))
    )
    selector_boss = (
        cq.Workplane("XZ")
        .circle(0.012)
        .extrude(0.006)
        .translate((0.032, gauge_case_front_y, -0.038))
    )
    gauge_shape = clamp_band.union(gauge_arm).union(gauge_case).union(selector_boss)

    plunger_top_collar = cq.Workplane("XY").circle(0.021).extrude(0.010)
    handle_shape = cq.Workplane("XY").circle(0.014).extrude(0.064).translate((0.0, 0.0, rod_exposed - 0.006))
    handle_crossbar = (
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(0.220, both=True)
        .translate((0.0, 0.0, rod_exposed + 0.030))
    )
    left_grip = (
        cq.Workplane("YZ")
        .circle(0.017)
        .extrude(0.045, both=True)
        .translate((-0.127, 0.0, rod_exposed + 0.030))
    )
    right_grip = (
        cq.Workplane("YZ")
        .circle(0.017)
        .extrude(0.045, both=True)
        .translate((0.127, 0.0, rod_exposed + 0.030))
    )

    knob_core = cq.Workplane("XZ").circle(0.008).extrude(0.006)
    knob_shape = knob_core.faces(">Y").workplane(centerOption="CenterOfMass").circle(0.016).extrude(0.013)

    base = model.part("base")
    base.visual(mesh_from_cadquery(base_shape, "base_shell"), material=base_mat, name="base_shell")

    barrel = model.part("barrel")
    barrel.visual(mesh_from_cadquery(barrel_shape, "barrel_shell"), material=steel_mat, name="barrel_shell")

    gauge_housing = model.part("gauge_housing")
    gauge_housing.visual(
        mesh_from_cadquery(gauge_shape, "gauge_housing_shell"),
        material=gauge_mat,
        name="gauge_housing_shell",
    )

    plunger = model.part("plunger")
    piston_shape = (
        cq.Workplane("XY")
        .cylinder(0.016, barrel_inner_r - 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, -0.255))
    )
    rod_shape = (
        cq.Workplane("XY").cylinder(
            rod_hidden + rod_exposed,
            rod_radius,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -rod_hidden))
        .union(plunger_top_collar)
        .union(piston_shape)
    )
    plunger.visual(mesh_from_cadquery(rod_shape, "plunger_rod"), material=steel_mat, name="plunger_rod")
    plunger.visual(mesh_from_cadquery(handle_shape, "plunger_boss"), material=steel_mat, name="plunger_boss")
    plunger.visual(mesh_from_cadquery(handle_crossbar, "handle_crossbar"), material=grip_mat, name="handle_crossbar")
    plunger.visual(mesh_from_cadquery(left_grip, "handle_left_grip"), material=grip_mat, name="handle_left_grip")
    plunger.visual(mesh_from_cadquery(right_grip, "handle_right_grip"), material=grip_mat, name="handle_right_grip")

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_cadquery(knob_shape, "selector_knob_shell"),
        material=knob_mat,
        name="selector_knob_shell",
    )

    model.articulation(
        "base_to_barrel",
        ArticulationType.FIXED,
        parent=base,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, base_height + base_pedestal_height)),
    )

    model.articulation(
        "barrel_to_gauge_housing",
        ArticulationType.FIXED,
        parent=barrel,
        child=gauge_housing,
        origin=Origin(xyz=(0.0, 0.0, gauge_mount_z)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, barrel_height + 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=130.0,
            velocity=0.65,
            lower=0.0,
            upper=plunger_travel,
        ),
    )

    model.articulation(
        "gauge_housing_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=gauge_housing,
        child=selector_knob,
        origin=Origin(xyz=(0.032, gauge_case_front_y + 0.006, -0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-1.25,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    barrel = object_model.get_part("barrel")
    gauge_housing = object_model.get_part("gauge_housing")
    plunger = object_model.get_part("plunger")
    selector_knob = object_model.get_part("selector_knob")

    plunger_joint = object_model.get_articulation("barrel_to_plunger")
    knob_joint = object_model.get_articulation("gauge_housing_to_selector_knob")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="barrel_shell",
        elem_b="plunger_rod",
        reason="The plunger rod is intentionally modeled as the captured sliding member inside the barrel sleeve.",
    )
    ctx.allow_overlap(
        barrel,
        gauge_housing,
        elem_a="barrel_shell",
        elem_b="gauge_housing_shell",
        reason="The front gauge clamp is modeled with a slight interference so the mesh-backed bracket remains visibly seated on the barrel.",
    )

    ctx.expect_contact(barrel, base, name="barrel seats on base")
    ctx.expect_contact(selector_knob, gauge_housing, name="selector knob mounts to gauge housing")

    rest_plunger_pos = ctx.part_world_position(plunger)
    rest_knob_pos = ctx.part_world_position(selector_knob)

    with ctx.pose({plunger_joint: plunger_joint.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem="plunger_rod",
            margin=0.002,
            name="plunger rod stays centered in barrel bore",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="plunger_rod",
            min_overlap=0.180,
            name="plunger rod retains insertion at full extension",
        )
        extended_plunger_pos = ctx.part_world_position(plunger)

    with ctx.pose({knob_joint: knob_joint.motion_limits.upper}):
        turned_knob_pos = ctx.part_world_position(selector_knob)
        ctx.expect_contact(
            selector_knob,
            gauge_housing,
            name="selector knob stays seated while rotating",
        )

    plunger_ok = (
        rest_plunger_pos is not None
        and extended_plunger_pos is not None
        and abs(extended_plunger_pos[0] - rest_plunger_pos[0]) < 1e-6
        and abs(extended_plunger_pos[1] - rest_plunger_pos[1]) < 1e-6
        and extended_plunger_pos[2] > rest_plunger_pos[2] + 0.20
    )
    ctx.check(
        "plunger translates upward along barrel axis",
        plunger_ok,
        details=f"rest={rest_plunger_pos}, extended={extended_plunger_pos}",
    )

    knob_ok = (
        rest_knob_pos is not None
        and turned_knob_pos is not None
        and abs(turned_knob_pos[0] - rest_knob_pos[0]) < 1e-6
        and abs(turned_knob_pos[1] - rest_knob_pos[1]) < 1e-6
        and abs(turned_knob_pos[2] - rest_knob_pos[2]) < 1e-6
    )
    ctx.check(
        "selector knob rotates about its own shaft without translating",
        knob_ok,
        details=f"rest={rest_knob_pos}, turned={turned_knob_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
