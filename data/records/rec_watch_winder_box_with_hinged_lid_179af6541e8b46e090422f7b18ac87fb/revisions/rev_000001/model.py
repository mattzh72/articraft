from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_slab_mesh(name: str, width: float, depth: float, height: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            height,
            cap=True,
            closed=True,
        ),
        name,
    )


def _rounded_frame_mesh(
    name: str,
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    height: float,
    *,
    outer_radius: float,
    inner_radius: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_width, outer_depth, outer_radius, corner_segments=12),
            [rounded_rect_profile(inner_width, inner_depth, inner_radius, corner_segments=12)],
            height,
            cap=True,
            center=False,
            closed=True,
        ),
        name,
    )


def _matte(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luxury_watch_winder_box")

    walnut = model.material("polished_walnut", rgba=(0.33, 0.16, 0.075, 1.0))
    lacquer = model.material("black_lacquer", rgba=(0.018, 0.016, 0.015, 1.0))
    velvet = model.material("charcoal_velvet", rgba=(0.018, 0.019, 0.022, 1.0))
    leather = model.material("saddle_leather", rgba=(0.45, 0.27, 0.13, 1.0))
    brass = model.material("brushed_brass", rgba=(0.82, 0.61, 0.30, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.76, 0.75, 0.70, 1.0))
    glass = model.material("smoked_glass", rgba=(0.35, 0.46, 0.50, 0.34))
    dial_glass = model.material("watch_crystal", rgba=(0.72, 0.86, 0.93, 0.45))
    rubber = model.material("soft_black_rubber", rgba=(0.028, 0.028, 0.030, 1.0))

    case = model.part("case")
    case.visual(
        _rounded_slab_mesh("case_bottom_slab", 0.420, 0.280, 0.030, 0.030),
        material=walnut,
        name="bottom_slab",
    )
    case.visual(
        Box((0.420, 0.018, 0.115)),
        origin=Origin(xyz=(0.0, -0.131, 0.0875)),
        material=walnut,
        name="front_wall",
    )
    case.visual(
        Box((0.420, 0.018, 0.115)),
        origin=Origin(xyz=(0.0, 0.131, 0.0875)),
        material=walnut,
        name="rear_wall",
    )
    case.visual(
        Box((0.018, 0.244, 0.115)),
        origin=Origin(xyz=(-0.201, 0.0, 0.0875)),
        material=walnut,
        name="side_wall_0",
    )
    case.visual(
        Box((0.018, 0.244, 0.115)),
        origin=Origin(xyz=(0.201, 0.0, 0.0875)),
        material=walnut,
        name="side_wall_1",
    )
    case.visual(
        Box((0.374, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.136, 0.149)),
        material=lacquer,
        name="top_lip",
    )
    case.visual(
        Box((0.374, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.136, 0.149)),
        material=lacquer,
        name="rear_top_lip",
    )
    case.visual(
        Box((0.014, 0.244, 0.008)),
        origin=Origin(xyz=(-0.203, 0.0, 0.149)),
        material=lacquer,
        name="side_top_lip_0",
    )
    case.visual(
        Box((0.014, 0.244, 0.008)),
        origin=Origin(xyz=(0.203, 0.0, 0.149)),
        material=lacquer,
        name="side_top_lip_1",
    )
    case.visual(
        Box((0.355, 0.210, 0.006)),
        origin=Origin(xyz=(0.0, -0.005, 0.034)),
        material=velvet,
        name="velvet_floor",
    )
    case.visual(
        Box((0.334, 0.012, 0.108)),
        origin=Origin(xyz=(0.0, 0.035, 0.088)),
        material=velvet,
        name="display_panel",
    )
    case.visual(
        Box((0.344, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.044, 0.144)),
        material=brass,
        name="upper_gallery_rail",
    )
    case.visual(
        Box((0.344, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.044, 0.034)),
        material=brass,
        name="lower_gallery_rail",
    )
    case.visual(
        Box((0.008, 0.008, 0.108)),
        origin=Origin(xyz=(-0.172, 0.044, 0.088)),
        material=brass,
        name="side_gallery_rail_0",
    )
    case.visual(
        Box((0.008, 0.008, 0.108)),
        origin=Origin(xyz=(0.172, 0.044, 0.088)),
        material=brass,
        name="side_gallery_rail_1",
    )
    case.visual(
        Box((0.320, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.096, 0.043)),
        material=lacquer,
        name="control_plinth",
    )
    case.visual(
        Box((0.300, 0.004, 0.005)),
        origin=Origin(xyz=(0.0, -0.142, 0.090)),
        material=brass,
        name="front_inlay",
    )
    for x in (-0.130, 0.130):
        case.visual(
            Cylinder(radius=0.008, length=0.088),
            origin=Origin(xyz=(x, 0.172, 0.154), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_steel,
            name=f"hinge_barrel_{0 if x < 0.0 else 1}",
        )
        case.visual(
            Box((0.082, 0.036, 0.0035)),
            origin=Origin(xyz=(x, 0.153, 0.148)),
            material=polished_steel,
            name=f"hinge_leaf_{0 if x < 0.0 else 1}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.404, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.288, 0.009)),
        material=lacquer,
        name="lid_frame",
    )
    lid.visual(
        Box((0.404, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.028, 0.009)),
        material=lacquer,
        name="rear_lid_rail",
    )
    lid.visual(
        Box((0.024, 0.260, 0.018)),
        origin=Origin(xyz=(-0.190, -0.158, 0.009)),
        material=lacquer,
        name="side_lid_rail_0",
    )
    lid.visual(
        Box((0.024, 0.260, 0.018)),
        origin=Origin(xyz=(0.190, -0.158, 0.009)),
        material=lacquer,
        name="side_lid_rail_1",
    )
    lid.visual(
        Box((0.366, 0.246, 0.004)),
        origin=Origin(xyz=(0.0, -0.158, 0.005)),
        material=glass,
        name="glass_panel",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.132, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, -0.012, 0.005)),
        material=polished_steel,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.112),
        origin=Origin(xyz=(0.0, -0.283, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="front_handle",
    )
    for x in (-0.046, 0.046):
        lid.visual(
            Cylinder(radius=0.0035, length=0.020),
            origin=Origin(xyz=(x, -0.277, 0.016)),
            material=brass,
            name=f"handle_post_{0 if x < 0.0 else 1}",
        )

    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(0.0, 0.172, 0.153)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    rotor_centers = [(-0.085, "0"), (0.085, "1")]
    for x, suffix in rotor_centers:
        rotor = model.part(f"winder_{suffix}")
        rotor.visual(
            Cylinder(radius=0.050, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=lacquer,
            name="outer_cup",
        )
        rotor.visual(
            Cylinder(radius=0.042, length=0.010),
            origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=velvet,
            name="recessed_dial",
        )
        rotor.visual(
            Box((0.058, 0.018, 0.034)),
            origin=Origin(xyz=(0.0, -0.030, 0.0)),
            material=leather,
            name="watch_cushion",
        )
        rotor.visual(
            Cylinder(radius=0.024, length=0.006),
            origin=Origin(xyz=(0.0, -0.042, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished_steel,
            name="watch_bezel",
        )
        rotor.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(0.0, -0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dial_glass,
            name="watch_face",
        )
        rotor.visual(
            Box((0.006, 0.004, 0.040)),
            origin=Origin(xyz=(0.0, -0.049, 0.012)),
            material=brass,
            name="dial_marker",
        )
        rotor.visual(
            Box((0.018, 0.008, 0.096)),
            origin=Origin(xyz=(0.0, -0.036, 0.0)),
            material=rubber,
            name="watch_strap",
        )
        model.articulation(
            f"case_to_winder_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=case,
            child=rotor,
            origin=Origin(xyz=(x, 0.015, 0.088)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=1.6),
        )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brass,
        name="selector_knob",
    )
    selector.visual(
        Box((0.004, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, 0.006, 0.011)),
        material=lacquer,
        name="selector_pointer",
    )
    model.articulation(
        "case_to_selector",
        ArticulationType.REVOLUTE,
        parent=case,
        child=selector,
        origin=Origin(xyz=(-0.060, -0.096, 0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=3.0, lower=-1.7, upper=1.7),
    )

    for index, x in enumerate((0.030, 0.070)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.022, 0.014, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=polished_steel,
            name="button_cap",
        )
        model.articulation(
            f"case_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=button,
            origin=Origin(xyz=(x, -0.096, 0.049)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.003),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    winder_0 = object_model.get_part("winder_0")
    winder_1 = object_model.get_part("winder_1")
    lid_joint = object_model.get_articulation("case_to_lid")
    winder_joint = object_model.get_articulation("case_to_winder_0")

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="top_lip",
        min_gap=0.0,
        max_gap=0.001,
        name="closed lid rests on the lacquered top lip",
    )
    for rotor in (winder_0, winder_1):
        ctx.expect_gap(
            case,
            rotor,
            axis="y",
            positive_elem="display_panel",
            negative_elem="outer_cup",
            min_gap=0.0,
            max_gap=0.002,
            name=f"{rotor.name} cup is seated against the display panel",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_marker = ctx.part_element_world_aabb(winder_0, elem="dial_marker")
    with ctx.pose({winder_joint: math.pi / 2.0}):
        spun_marker = ctx.part_element_world_aabb(winder_0, elem="dial_marker")
    if rest_marker is not None and spun_marker is not None:
        rest_center = tuple((rest_marker[0][i] + rest_marker[1][i]) * 0.5 for i in range(3))
        spun_center = tuple((spun_marker[0][i] + spun_marker[1][i]) * 0.5 for i in range(3))
        moved = abs(spun_center[0] - rest_center[0]) > 0.008 and abs(spun_center[2] - rest_center[2]) > 0.008
    else:
        moved = False
        rest_center = spun_center = None
    ctx.check(
        "winding cup visibly rotates around its motor axis",
        moved,
        details=f"rest_marker={rest_center}, spun_marker={spun_center}",
    )

    return ctx.report()


object_model = build_object_model()
