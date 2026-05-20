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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, thickness: float):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        center=True,
    ).rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="washer_dryer_combo")

    enamel = model.material("appliance_white", rgba=(0.95, 0.96, 0.97, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.15, 0.18, 0.22, 0.38))
    rubber = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    detergent_blue = model.material("detergent_blue", rgba=(0.62, 0.75, 0.90, 1.0))
    indicator = model.material("indicator_black", rgba=(0.06, 0.07, 0.08, 1.0))

    body_w = 0.60
    body_d = 0.68
    body_h = 0.85
    front_y = body_d / 2.0
    wall_t = 0.022
    top_t = 0.025
    rear_t = 0.018
    base_t = 0.035
    fascia_t = 0.018

    door_center_z = 0.43
    cabinet_bezel_outer_r = 0.257
    cabinet_bezel_inner_r = 0.205
    door_outer_r = 0.237
    door_ring_inner_r = 0.178
    door_glass_r = 0.182
    door_t = 0.038

    drawer_travel = 0.12
    drawer_center_x = -0.1685
    drawer_center_z = 0.778
    drawer_panel_w = 0.215
    drawer_panel_h = 0.088
    drawer_panel_t = 0.020
    drawer_bin_w = 0.194
    drawer_bin_h = 0.070
    drawer_bin_d = 0.245
    drawer_housing_d = 0.276

    cabinet_bezel_mesh = _annulus_mesh(
        "washer_front_bezel",
        outer_radius=cabinet_bezel_outer_r,
        inner_radius=cabinet_bezel_inner_r,
        thickness=fascia_t,
    )
    door_ring_mesh = _annulus_mesh(
        "washer_door_ring",
        outer_radius=door_outer_r,
        inner_radius=door_ring_inner_r,
        thickness=door_t,
    )
    door_inner_trim_mesh = _annulus_mesh(
        "washer_door_inner_trim",
        outer_radius=0.193,
        inner_radius=0.148,
        thickness=0.014,
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((body_w, body_d, top_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - top_t / 2.0)),
        material=enamel,
        name="top_cover",
    )
    cabinet.visual(
        Box((wall_t, body_d, body_h - top_t)),
        origin=Origin(xyz=(-body_w / 2.0 + wall_t / 2.0, 0.0, (body_h - top_t) / 2.0)),
        material=enamel,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall_t, body_d, body_h - top_t)),
        origin=Origin(xyz=(body_w / 2.0 - wall_t / 2.0, 0.0, (body_h - top_t) / 2.0)),
        material=enamel,
        name="right_wall",
    )
    cabinet.visual(
        Box((body_w - 2.0 * wall_t, rear_t, body_h - top_t - base_t)),
        origin=Origin(
            xyz=(
                0.0,
                -front_y + rear_t / 2.0,
                base_t + (body_h - top_t - base_t) / 2.0,
            )
        ),
        material=enamel,
        name="rear_panel",
    )
    cabinet.visual(
        Box((body_w - 2.0 * wall_t, body_d - 0.10, base_t)),
        origin=Origin(xyz=(0.0, -0.02, base_t / 2.0)),
        material=enamel,
        name="base_floor",
    )
    cabinet.visual(
        Box((body_w - 2.0 * wall_t, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, front_y - 0.05, 0.07)),
        material=enamel,
        name="front_plinth",
    )
    cabinet.visual(
        Box((0.048, fascia_t, 0.592)),
        origin=Origin(xyz=(-0.276, front_y - fascia_t / 2.0, 0.436)),
        material=enamel,
        name="front_left_cheek",
    )
    cabinet.visual(
        Box((0.048, fascia_t, 0.592)),
        origin=Origin(xyz=(0.276, front_y - fascia_t / 2.0, 0.436)),
        material=enamel,
        name="front_right_cheek",
    )
    cabinet.visual(
        Box((0.510, fascia_t, 0.178)),
        origin=Origin(xyz=(0.0, front_y - fascia_t / 2.0, 0.089)),
        material=enamel,
        name="lower_front_band",
    )
    cabinet.visual(
        Box((0.510, fascia_t, 0.056)),
        origin=Origin(xyz=(0.0, front_y - fascia_t / 2.0, 0.706)),
        material=enamel,
        name="upper_front_band",
    )
    cabinet.visual(
        cabinet_bezel_mesh,
        origin=Origin(xyz=(0.0, front_y - fascia_t / 2.0, door_center_z)),
        material=enamel,
        name="front_bezel",
    )
    cabinet.visual(
        Box((0.010, fascia_t, 0.094)),
        origin=Origin(xyz=(-0.283, front_y - fascia_t / 2.0, drawer_center_z)),
        material=enamel,
        name="drawer_left_border",
    )
    cabinet.visual(
        Box((0.028, fascia_t, 0.120)),
        origin=Origin(xyz=(-0.045, front_y - fascia_t / 2.0, 0.790)),
        material=enamel,
        name="drawer_separator",
    )
    cabinet.visual(
        Box((0.314, fascia_t, 0.120)),
        origin=Origin(xyz=(0.126, front_y - fascia_t / 2.0, 0.790)),
        material=enamel,
        name="control_right_fascia",
    )
    cabinet.visual(
        Box((0.018, 0.016, 0.220)),
        origin=Origin(xyz=(-0.246, 0.340, door_center_z)),
        material=graphite,
        name="hinge_mount",
    )
    cabinet.visual(
        Cylinder(radius=0.206, length=0.060),
        origin=Origin(
            xyz=(0.0, 0.295, door_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="door_boot",
    )
    cabinet.visual(
        Cylinder(radius=0.178, length=0.440),
        origin=Origin(
            xyz=(0.0, 0.100, door_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="drum_shell",
    )
    cabinet.visual(
        Box((0.219, drawer_housing_d, 0.012)),
        origin=Origin(xyz=(drawer_center_x, 0.185, 0.728)),
        material=enamel,
        name="drawer_housing_floor",
    )
    cabinet.visual(
        Box((0.219, drawer_housing_d, 0.010)),
        origin=Origin(xyz=(drawer_center_x, 0.185, 0.820)),
        material=enamel,
        name="drawer_housing_roof",
    )
    cabinet.visual(
        Box((0.012, drawer_housing_d, 0.092)),
        origin=Origin(xyz=(drawer_center_x - 0.110, 0.185, drawer_center_z)),
        material=enamel,
        name="drawer_housing_left",
    )
    cabinet.visual(
        Box((0.012, drawer_housing_d, 0.092)),
        origin=Origin(xyz=(drawer_center_x + 0.110, 0.185, drawer_center_z)),
        material=enamel,
        name="drawer_housing_right",
    )
    cabinet.visual(
        Box((0.219, 0.012, 0.092)),
        origin=Origin(xyz=(drawer_center_x, 0.053, drawer_center_z)),
        material=enamel,
        name="drawer_housing_stop",
    )
    cabinet.visual(
        Box((0.116, 0.006, 0.040)),
        origin=Origin(xyz=(0.045, 0.337, 0.810)),
        material=indicator,
        name="display_window",
    )
    cabinet.visual(
        Cylinder(radius=0.035, length=0.018),
        origin=Origin(
            xyz=(0.205, 0.335, 0.790),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="program_dial",
    )
    cabinet.visual(
        Box((0.032, 0.012, 0.012)),
        origin=Origin(xyz=(0.115, 0.336, 0.760)),
        material=graphite,
        name="start_button",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    door = model.part("door")
    door.visual(
        door_ring_mesh,
        origin=Origin(xyz=(door_outer_r, 0.0, 0.0)),
        material=enamel,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=door_glass_r, length=0.018),
        origin=Origin(
            xyz=(door_outer_r, 0.004, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_glass,
        name="door_glass",
    )
    door.visual(
        door_inner_trim_mesh,
        origin=Origin(xyz=(door_outer_r, -0.010, 0.0)),
        material=graphite,
        name="door_inner_trim",
    )
    door.visual(
        Box((0.052, 0.044, 0.340)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=graphite,
        name="hinge_bracket",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=graphite,
        name="upper_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.146)),
        material=graphite,
        name="lower_hinge_barrel",
    )
    door.visual(
        Box((0.032, 0.040, 0.102)),
        origin=Origin(xyz=(0.405, 0.018, 0.0)),
        material=graphite,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.474, 0.060, 0.474)),
        mass=5.8,
        origin=Origin(xyz=(0.237, 0.0, 0.0)),
    )

    drawer = model.part("soap_drawer")
    drawer.visual(
        Box((drawer_panel_w, drawer_panel_t, drawer_panel_h)),
        origin=Origin(xyz=(0.0, -drawer_panel_t / 2.0, 0.0)),
        material=enamel,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_bin_w, drawer_bin_d, drawer_bin_h)),
        origin=Origin(xyz=(0.0, -0.1405, 0.0)),
        material=enamel,
        name="drawer_bin",
    )
    drawer.visual(
        Box((0.006, 0.170, 0.056)),
        origin=Origin(xyz=(-0.030, -0.120, 0.0)),
        material=graphite,
        name="drawer_divider_major",
    )
    drawer.visual(
        Box((0.006, 0.100, 0.056)),
        origin=Origin(xyz=(0.042, -0.090, 0.0)),
        material=graphite,
        name="drawer_divider_minor",
    )
    drawer.visual(
        Box((0.060, 0.120, 0.030)),
        origin=Origin(xyz=(-0.062, -0.115, -0.020)),
        material=detergent_blue,
        name="drawer_softener_insert",
    )
    drawer.visual(
        Box((0.070, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
        material=graphite,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_panel_w, drawer_bin_d + drawer_panel_t, drawer_panel_h)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.1225, 0.0)),
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-door_outer_r, front_y + 0.030, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(drawer_center_x, front_y, drawer_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.25,
            lower=0.0,
            upper=drawer_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drawer = object_model.get_part("soap_drawer")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drawer_slide = object_model.get_articulation("cabinet_to_drawer")

    ctx.check(
        "door hinge uses vertical axis",
        tuple(round(value, 3) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "soap drawer slides forward along +Y",
        tuple(round(value, 3) for value in drawer_slide.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drawer_slide.axis}",
    )

    with ctx.pose({door_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_ring",
            negative_elem="front_bezel",
            min_gap=0.0,
            max_gap=0.020,
            name="closed door sits just proud of the cabinet face",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.40,
            name="closed door covers the drum opening",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_bin",
            elem_b="drawer_housing_floor",
            min_overlap=0.20,
            name="closed soap drawer remains deeply inserted in its housing",
        )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: math.radians(105.0)}):
        open_door_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "porthole door swings outward on its side hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10
        and open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.02,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_bin",
            elem_b="drawer_housing_floor",
            min_overlap=0.20,
            name="drawer tray is retained inside the housing at rest",
        )
    with ctx.pose({drawer_slide: 0.12}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_bin",
            elem_b="drawer_housing_floor",
            min_overlap=0.12,
            name="extended drawer keeps retained insertion in the housing",
        )

    ctx.check(
        "soap drawer translates outward when opened",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.10,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
