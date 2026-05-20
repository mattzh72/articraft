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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    *,
    x: float,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_espresso_machine")

    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    black_bakelite = model.material("black_bakelite", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_black = model.material("tray_black", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.30, 0.33)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
    )

    body_shell = section_loft(
        [
            _yz_section(x=0.160, width=0.220, height=0.250, radius=0.062, z_center=0.145),
            _yz_section(x=0.080, width=0.280, height=0.300, radius=0.078, z_center=0.160),
            _yz_section(x=-0.010, width=0.310, height=0.320, radius=0.090, z_center=0.166),
            _yz_section(x=-0.100, width=0.310, height=0.310, radius=0.086, z_center=0.163),
            _yz_section(x=-0.170, width=0.270, height=0.280, radius=0.072, z_center=0.156),
        ]
    )
    body.visual(
        _save_mesh("body_shell", body_shell),
        material=chrome,
        name="body_shell",
    )
    body.visual(
        Box((0.262, 0.262, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_steel,
        name="base_plinth",
    )
    for x_pos in (-0.095, 0.095):
        for y_pos in (-0.095, 0.095):
            body.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(xyz=(x_pos, y_pos, 0.005)),
                material=black_bakelite,
                name=f"foot_{'f' if x_pos > 0 else 'r'}_{'r' if y_pos > 0 else 'l'}",
            )
    body.visual(
        Box((0.145, 0.180, 0.006)),
        origin=Origin(xyz=(0.085, 0.0, 0.031)),
        material=tray_black,
        name="drip_tray",
    )
    body.visual(
        Box((0.145, 0.012, 0.016)),
        origin=Origin(xyz=(0.153, 0.0, 0.036)),
        material=dark_steel,
        name="tray_lip",
    )
    body.visual(
        Box((0.180, 0.130, 0.008)),
        origin=Origin(xyz=(-0.010, 0.0, 0.314)),
        material=dark_steel,
        name="top_deck",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(-0.023, 0.0, 0.323)),
        material=dark_steel,
        name="fill_neck",
    )
    body.visual(
        Box((0.070, 0.070, 0.055)),
        origin=Origin(xyz=(0.125, 0.0, 0.220)),
        material=chrome,
        name="group_block",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.052),
        origin=Origin(xyz=(0.166, 0.0, 0.222), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="group_barrel",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.186, 0.0, 0.212)),
        material=dark_steel,
        name="group_collar",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.186, 0.0, 0.200)),
        material=dark_steel,
        name="brew_spout",
    )
    body.visual(
        Box((0.034, 0.022, 0.050)),
        origin=Origin(xyz=(0.076, 0.151, 0.210)),
        material=dark_steel,
        name="wand_support",
    )
    body.visual(
        Box((0.026, 0.018, 0.014)),
        origin=Origin(xyz=(0.018, 0.155, 0.246)),
        material=dark_steel,
        name="valve_mount",
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=dark_steel,
        name="basket_body",
    )
    portafilter.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=chrome,
        name="basket_rim",
    )
    portafilter.visual(
        Box((0.012, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.029, -0.003)),
        material=dark_steel,
        name="lug_0",
    )
    portafilter.visual(
        Box((0.012, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.029, -0.003)),
        material=dark_steel,
        name="lug_1",
    )
    portafilter.visual(
        Box((0.080, 0.020, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, -0.017)),
        material=dark_steel,
        name="handle_neck",
    )
    portafilter_handle = tube_from_spline_points(
        [
            (0.034, 0.0, -0.014),
            (0.072, 0.0, -0.016),
            (0.120, 0.0, -0.021),
            (0.172, 0.0, -0.030),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    portafilter.visual(
        _save_mesh("portafilter_handle", portafilter_handle),
        material=black_bakelite,
        name="handle_grip",
    )
    portafilter.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.178, 0.0, -0.031)),
        material=black_bakelite,
        name="handle_tip",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.210, 0.070, 0.070)),
        mass=0.8,
        origin=Origin(xyz=(0.095, 0.0, -0.018)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(),
        material=chrome,
        name="wand_collar",
    )
    wand_tube = tube_from_spline_points(
        [
            (0.0, 0.0, -0.008),
            (-0.006, 0.014, -0.020),
            (-0.014, 0.028, -0.064),
            (-0.024, 0.032, -0.114),
            (-0.022, 0.028, -0.154),
        ],
        radius=0.0042,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    steam_wand.visual(
        _save_mesh("steam_wand_tube", wand_tube),
        material=chrome,
        name="wand_tube",
    )
    steam_wand.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(-0.022, 0.028, -0.154)),
        material=dark_steel,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.190)),
        mass=0.12,
        origin=Origin(xyz=(-0.014, 0.018, -0.080)),
    )

    fill_cap = model.part("fill_cap")
    fill_cap.visual(
        Cylinder(radius=0.005, length=0.058),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    fill_cap.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=chrome,
        name="cap_plate",
    )
    fill_cap.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.044, 0.0, 0.010)),
        material=knob_black,
        name="cap_knob",
    )
    fill_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.020),
        mass=0.10,
        origin=Origin(xyz=(0.034, 0.0, 0.003)),
    )

    valve_lever = model.part("valve_lever")
    valve_lever.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lever_hub",
    )
    lever_arm = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.010, 0.004),
            (0.0, 0.020, 0.013),
        ],
        radius=0.0036,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    valve_lever.visual(
        _save_mesh("valve_lever_arm", lever_arm),
        material=black_bakelite,
        name="lever_arm",
    )
    valve_lever.visual(
        Sphere(radius=0.0065),
        origin=Origin(xyz=(0.0, 0.022, 0.015)),
        material=knob_black,
        name="lever_tip",
    )
    valve_lever.inertial = Inertial.from_geometry(
        Box((0.018, 0.032, 0.028)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.011, 0.008)),
    )

    model.articulation(
        "portafilter_lock",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.194, 0.0, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "steam_wand_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.076, 0.169, 0.222)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-0.65,
            upper=0.75,
        ),
    )
    model.articulation(
        "fill_cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=fill_cap,
        origin=Origin(xyz=(-0.055, 0.0, 0.334)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=1.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "valve_control",
        ArticulationType.REVOLUTE,
        parent=body,
        child=valve_lever,
        origin=Origin(xyz=(0.018, 0.170, 0.246)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=-0.45,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    fill_cap = object_model.get_part("fill_cap")
    valve_lever = object_model.get_part("valve_lever")

    portafilter_lock = object_model.get_articulation("portafilter_lock")
    steam_wand_pivot = object_model.get_articulation("steam_wand_pivot")
    fill_cap_hinge = object_model.get_articulation("fill_cap_hinge")
    valve_control = object_model.get_articulation("valve_control")

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="group_collar",
        negative_elem="basket_rim",
        name="portafilter seats just below the group head",
    )
    ctx.expect_overlap(
        body,
        portafilter,
        axes="xy",
        min_overlap=0.040,
        elem_a="group_collar",
        elem_b="basket_rim",
        name="portafilter stays centered under the brew axis",
    )
    ctx.expect_gap(
        fill_cap,
        body,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="cap_plate",
        negative_elem="fill_neck",
        name="fill cap rests neatly on the filler neck",
    )

    rest_handle = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle_tip"))
    with ctx.pose({portafilter_lock: portafilter_lock.motion_limits.upper}):
        turned_handle = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle_tip"))
    ctx.check(
        "portafilter rotates about the vertical brew axis",
        rest_handle is not None
        and turned_handle is not None
        and turned_handle[1] > rest_handle[1] + 0.07,
        details=f"rest_handle={rest_handle}, turned_handle={turned_handle}",
    )

    rest_tip = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    with ctx.pose({steam_wand_pivot: steam_wand_pivot.motion_limits.upper}):
        swung_tip = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    ctx.check(
        "steam wand swings laterally on its side pivot",
        rest_tip is not None
        and swung_tip is not None
        and ((swung_tip[0] - rest_tip[0]) ** 2 + (swung_tip[1] - rest_tip[1]) ** 2) ** 0.5 > 0.02,
        details=f"rest_tip={rest_tip}, swung_tip={swung_tip}",
    )

    rest_cap = _aabb_center(ctx.part_element_world_aabb(fill_cap, elem="cap_knob"))
    with ctx.pose({fill_cap_hinge: fill_cap_hinge.motion_limits.upper}):
        open_cap = _aabb_center(ctx.part_element_world_aabb(fill_cap, elem="cap_knob"))
        ctx.expect_gap(
            fill_cap,
            body,
            axis="z",
            min_gap=0.020,
            positive_elem="cap_knob",
            negative_elem="fill_neck",
            name="opened fill cap knob clears the filler neck",
        )
    ctx.check(
        "fill cap lifts upward on the rear hinge",
        rest_cap is not None
        and open_cap is not None
        and open_cap[2] > rest_cap[2] + 0.03,
        details=f"rest_cap={rest_cap}, open_cap={open_cap}",
    )

    rest_lever = _aabb_center(ctx.part_element_world_aabb(valve_lever, elem="lever_tip"))
    with ctx.pose({valve_control: valve_control.motion_limits.upper}):
        open_lever = _aabb_center(ctx.part_element_world_aabb(valve_lever, elem="lever_tip"))
    ctx.check(
        "valve lever pivots about its side-facing axis",
        rest_lever is not None
        and open_lever is not None
        and open_lever[2] > rest_lever[2] + 0.01,
        details=f"rest_lever={rest_lever}, open_lever={open_lever}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
