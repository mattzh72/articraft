from __future__ import annotations

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


BODY_LIFT = 0.15
LID_OPEN = 1.28
BUTTON_TRAVEL = 0.004


def rounded_button(length: float, width: float, height: float, fillet: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z")
        .fillet(fillet)
        .faces(">Z or <Z")
        .edges()
        .fillet(fillet * 0.55)
    )


def build_kettle_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .ellipse(0.072, 0.056)
        .workplane(offset=0.055)
        .ellipse(0.094, 0.074)
        .workplane(offset=0.072)
        .ellipse(0.089, 0.070)
        .workplane(offset=0.078)
        .ellipse(0.060, 0.048)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.010)
        .ellipse(0.058, 0.044)
        .workplane(offset=0.048)
        .ellipse(0.080, 0.061)
        .workplane(offset=0.064)
        .ellipse(0.075, 0.057)
        .workplane(offset=0.060)
        .ellipse(0.043, 0.034)
        .loft(combine=True)
    )
    top_opening = (
        cq.Workplane("XY")
        .workplane(offset=0.186)
        .ellipse(0.052, 0.040)
        .extrude(0.050)
    )
    underside_socket = cq.Workplane("XY").circle(0.029).extrude(0.018)
    spout = (
        cq.Workplane("YZ")
        .ellipse(0.022, 0.017)
        .extrude(0.050)
        .translate((0.078, 0.0, 0.125))
    )
    spout_opening = (
        cq.Workplane("YZ")
        .ellipse(0.012, 0.009)
        .extrude(0.052)
        .translate((0.080, 0.0, 0.124))
    )
    return outer.cut(inner).cut(top_opening).cut(underside_socket).union(spout).cut(spout_opening)


def build_handle() -> cq.Workplane:
    outer_wire = [
        (-0.150, 0.040),
        (-0.150, 0.184),
        (-0.138, 0.214),
        (-0.106, 0.230),
        (-0.084, 0.214),
        (-0.087, 0.188),
        (-0.101, 0.165),
        (-0.106, 0.076),
        (-0.095, 0.060),
        (-0.084, 0.040),
    ]
    inner_wire = [
        (-0.131, 0.067),
        (-0.131, 0.176),
        (-0.121, 0.198),
        (-0.106, 0.208),
        (-0.103, 0.184),
        (-0.115, 0.160),
        (-0.119, 0.082),
        (-0.108, 0.068),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(outer_wire)
        .close()
        .polyline(inner_wire)
        .close()
        .extrude(0.034, both=True)
        .edges("|Y")
        .fillet(0.007)
    )


def build_base_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse(0.128, 0.102)
        .workplane(offset=0.012)
        .ellipse(0.121, 0.096)
        .workplane(offset=0.008)
        .ellipse(0.114, 0.090)
        .loft(combine=True)
    )


def build_socket_ring() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.022)
        .extrude(0.008)
        .faces(">Z")
        .edges()
        .fillet(0.0025)
    )


def build_lid_panel() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse(0.060, 0.048)
        .workplane(offset=0.010)
        .ellipse(0.052, 0.042)
        .workplane(offset=0.009)
        .ellipse(0.035, 0.028)
        .loft(combine=True)
    )


def build_power_switch() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.020, 0.010, 0.014)
        .edges("|Z")
        .fillet(0.0025)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_kettle")

    brushed_steel = model.material("brushed_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.19, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))
    amber = model.material("amber", rgba=(0.92, 0.55, 0.12, 1.0))
    blue = model.material("button_blue", rgba=(0.26, 0.50, 0.82, 1.0))
    mint = model.material("button_mint", rgba=(0.38, 0.74, 0.62, 1.0))
    cream = model.material("button_cream", rgba=(0.88, 0.85, 0.73, 1.0))

    heating_base = model.part("heating_base")
    heating_base.visual(
        mesh_from_cadquery(build_base_shell(), "heating_base_shell"),
        material=graphite,
        name="base_shell",
    )
    heating_base.visual(
        mesh_from_cadquery(build_socket_ring(), "heating_base_socket"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=soft_black,
        name="socket_ring",
    )

    kettle = model.part("kettle")
    kettle.visual(
        mesh_from_cadquery(build_kettle_shell(), "kettle_shell"),
        material=brushed_steel,
        name="shell",
    )
    kettle.visual(
        mesh_from_cadquery(build_handle(), "kettle_handle"),
        material=graphite,
        name="handle",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(build_lid_panel(), "kettle_lid"),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=graphite,
        name="lid_panel",
    )

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_cadquery(rounded_button(0.026, 0.017, 0.007, 0.003), "release_button"),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=soft_black,
        name="release_cap",
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        mesh_from_cadquery(build_power_switch(), "power_switch"),
        origin=Origin(xyz=(-0.010, 0.0, -0.007)),
        material=amber,
        name="switch_rocker",
    )

    temperature_specs = (
        ("temp_button_0", -0.030, blue),
        ("temp_button_1", 0.000, cream),
        ("temp_button_2", 0.030, mint),
    )
    for index, (part_name, y_pos, material) in enumerate(temperature_specs):
        button = model.part(part_name)
        button.visual(
            mesh_from_cadquery(rounded_button(0.022, 0.016, 0.008, 0.003), f"base_temp_button_{index}"),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"base_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=heating_base,
            child=button,
            origin=Origin(xyz=(0.094, y_pos, 0.020)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
        )

    model.articulation(
        "base_to_kettle",
        ArticulationType.PRISMATIC,
        parent=heating_base,
        child=kettle,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.40, lower=0.0, upper=BODY_LIFT),
    )
    model.articulation(
        "kettle_to_lid",
        ArticulationType.REVOLUTE,
        parent=kettle,
        child=lid,
        origin=Origin(xyz=(-0.047, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=LID_OPEN),
    )
    model.articulation(
        "kettle_to_release_button",
        ArticulationType.PRISMATIC,
        parent=kettle,
        child=release_button,
        origin=Origin(xyz=(-0.118, 0.0, 0.224)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    model.articulation(
        "kettle_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=kettle,
        child=power_switch,
        origin=Origin(xyz=(-0.147, 0.0, 0.061)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=-0.25, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    heating_base = object_model.get_part("heating_base")
    kettle = object_model.get_part("kettle")
    lid = object_model.get_part("lid")
    release_button = object_model.get_part("release_button")
    power_switch = object_model.get_part("power_switch")
    lift_joint = object_model.get_articulation("base_to_kettle")
    lid_joint = object_model.get_articulation("kettle_to_lid")
    release_joint = object_model.get_articulation("kettle_to_release_button")
    switch_joint = object_model.get_articulation("kettle_to_power_switch")

    ctx.expect_origin_gap(
        kettle,
        heating_base,
        axis="z",
        min_gap=0.019,
        max_gap=0.021,
        name="kettle seats just above the heating base frame",
    )
    ctx.expect_overlap(
        kettle,
        heating_base,
        axes="xy",
        elem_a="shell",
        elem_b="base_shell",
        min_overlap=0.14,
        name="kettle body sits over the heating base footprint",
    )
    ctx.expect_overlap(
        lid,
        kettle,
        axes="xy",
        elem_a="lid_panel",
        elem_b="shell",
        min_overlap=0.07,
        name="closed lid covers the kettle opening",
    )
    ctx.expect_contact(
        release_button,
        kettle,
        elem_a="release_cap",
        elem_b="handle",
        name="release button is mounted on the handle crown",
    )

    for part_name in ("temp_button_0", "temp_button_1", "temp_button_2"):
        button = object_model.get_part(part_name)
        ctx.expect_contact(
            button,
            heating_base,
            elem_a="button_cap",
            elem_b="base_shell",
            name=f"{part_name} sits on the base top",
        )

    rest_kettle_pos = ctx.part_world_position(kettle)
    lifted_kettle_pos = None
    lift_upper = lift_joint.motion_limits.upper if lift_joint.motion_limits is not None else None
    if lift_upper is not None:
        with ctx.pose({lift_joint: lift_upper}):
            lifted_kettle_pos = ctx.part_world_position(kettle)
    ctx.check(
        "kettle lifts vertically off the base",
        rest_kettle_pos is not None
        and lifted_kettle_pos is not None
        and lifted_kettle_pos[2] > rest_kettle_pos[2] + 0.10,
        details=f"rest={rest_kettle_pos}, lifted={lifted_kettle_pos}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    open_lid_aabb = None
    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    if lid_upper is not None:
        with ctx.pose({lid_joint: lid_upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.055,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    release_rest = ctx.part_world_position(release_button)
    release_pressed = None
    release_upper = release_joint.motion_limits.upper if release_joint.motion_limits is not None else None
    if release_upper is not None:
        with ctx.pose({release_joint: release_upper}):
            release_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release button depresses into the handle",
        release_rest is not None
        and release_pressed is not None
        and release_pressed[2] < release_rest[2] - 0.004,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    switch_rest = ctx.part_element_world_aabb(power_switch, elem="switch_rocker")
    switch_on = None
    switch_upper = switch_joint.motion_limits.upper if switch_joint.motion_limits is not None else None
    if switch_upper is not None:
        with ctx.pose({switch_joint: switch_upper}):
            switch_on = ctx.part_element_world_aabb(power_switch, elem="switch_rocker")
    ctx.check(
        "power switch pivots on its lower handle mount",
        switch_rest is not None
        and switch_on is not None
        and switch_on[1][2] > switch_rest[1][2] + 0.003,
        details=f"rest={switch_rest}, switched={switch_on}",
    )

    for part_name in ("temp_button_0", "temp_button_1", "temp_button_2"):
        joint = object_model.get_articulation(f"base_to_{part_name}")
        button = object_model.get_part(part_name)
        rest_pos = ctx.part_world_position(button)
        pressed_pos = None
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is not None:
            with ctx.pose({joint: upper}):
                pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{part_name} depresses",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.0025,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
