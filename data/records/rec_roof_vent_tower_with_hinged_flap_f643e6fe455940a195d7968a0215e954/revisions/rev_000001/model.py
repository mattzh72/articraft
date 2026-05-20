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


def _rect_loop(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _build_outlet_frame_mesh(
    *,
    outer_width: float,
    outer_height: float,
    opening_width: float,
    opening_height: float,
    depth: float,
):
    frame = ExtrudeWithHolesGeometry(
        _rect_loop(outer_height, outer_width),
        [_rect_loop(opening_height, opening_width)],
        depth,
        center=True,
        cap=True,
        closed=True,
    )
    frame.rotate_y(math.pi / 2.0)
    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    shell = model.material("shell_galvanized", rgba=(0.77, 0.79, 0.80, 1.0))
    frame_finish = model.material("frame_galvanized", rgba=(0.69, 0.71, 0.73, 1.0))
    flashing = model.material("flashing_dark", rgba=(0.34, 0.35, 0.37, 1.0))
    flap_finish = model.material("flap_aluminum", rgba=(0.58, 0.60, 0.63, 1.0))
    hinge_finish = model.material("hinge_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    flange_t = 0.010
    curb_h = 0.085
    tower_w = 0.380
    tower_d = 0.300
    tower_h = 0.570
    curb_w = tower_w
    curb_d = tower_d
    wall_t = 0.014
    overlap = 0.002

    opening_w = 0.280
    opening_h = 0.200
    opening_bottom = flange_t + curb_h + 0.225
    opening_center_z = opening_bottom + opening_h * 0.5

    frame_face = 0.020
    frame_depth = 0.018
    frame_outer_w = opening_w + 2.0 * frame_face
    frame_outer_h = opening_h + 2.0 * frame_face

    tower = model.part("tower_housing")
    tower.visual(
        Box((0.500, 0.400, flange_t)),
        origin=Origin(xyz=(0.0, 0.0, flange_t * 0.5)),
        material=flashing,
        name="roof_flange",
    )

    curb_center_z = flange_t + curb_h * 0.5 - overlap * 0.5
    tower.visual(
        Box((curb_d, wall_t, curb_h)),
        origin=Origin(xyz=(0.0, curb_w * 0.5 - wall_t * 0.5, curb_center_z)),
        material=shell,
        name="curb_left",
    )
    tower.visual(
        Box((curb_d, wall_t, curb_h)),
        origin=Origin(xyz=(0.0, -curb_w * 0.5 + wall_t * 0.5, curb_center_z)),
        material=shell,
        name="curb_right",
    )
    tower.visual(
        Box((wall_t, curb_w - wall_t, curb_h)),
        origin=Origin(xyz=(-curb_d * 0.5 + wall_t * 0.5, 0.0, curb_center_z)),
        material=shell,
        name="curb_back",
    )
    tower.visual(
        Box((wall_t, curb_w - wall_t, curb_h)),
        origin=Origin(xyz=(curb_d * 0.5 - wall_t * 0.5, 0.0, curb_center_z)),
        material=shell,
        name="curb_front",
    )

    tower_base_z = flange_t + curb_h - overlap
    tower_center_z = tower_base_z + tower_h * 0.5
    front_x = tower_d * 0.5 - wall_t * 0.5

    tower.visual(
        Box((tower_d, wall_t, tower_h)),
        origin=Origin(xyz=(0.0, tower_w * 0.5 - wall_t * 0.5, tower_center_z)),
        material=shell,
        name="left_wall",
    )
    tower.visual(
        Box((tower_d, wall_t, tower_h)),
        origin=Origin(xyz=(0.0, -tower_w * 0.5 + wall_t * 0.5, tower_center_z)),
        material=shell,
        name="right_wall",
    )
    tower.visual(
        Box((wall_t, tower_w - wall_t, tower_h)),
        origin=Origin(xyz=(-tower_d * 0.5 + wall_t * 0.5, 0.0, tower_center_z)),
        material=shell,
        name="back_wall",
    )

    lower_front_h = opening_bottom - tower_base_z
    upper_front_h = tower_base_z + tower_h - (opening_bottom + opening_h)
    jamb_w = (tower_w - opening_w) * 0.5

    tower.visual(
        Box((wall_t, tower_w, lower_front_h + overlap)),
        origin=Origin(
            xyz=(
                front_x,
                0.0,
                tower_base_z + (lower_front_h + overlap) * 0.5,
            )
        ),
        material=shell,
        name="front_skirt",
    )
    tower.visual(
        Box((wall_t, tower_w, upper_front_h + overlap)),
        origin=Origin(
            xyz=(
                front_x,
                0.0,
                opening_bottom + opening_h + (upper_front_h + overlap) * 0.5 - overlap,
            )
        ),
        material=shell,
        name="front_cap",
    )
    tower.visual(
        Box((wall_t, jamb_w + overlap, opening_h + 2.0 * overlap)),
        origin=Origin(
            xyz=(
                front_x,
                opening_w * 0.5 + (jamb_w + overlap) * 0.5 - overlap,
                opening_center_z,
            )
        ),
        material=shell,
        name="left_jamb",
    )
    tower.visual(
        Box((wall_t, jamb_w + overlap, opening_h + 2.0 * overlap)),
        origin=Origin(
            xyz=(
                front_x,
                -opening_w * 0.5 - (jamb_w + overlap) * 0.5 + overlap,
                opening_center_z,
            )
        ),
        material=shell,
        name="right_jamb",
    )

    tower.visual(
        Box((tower_d + 0.010, tower_w + 0.020, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, tower_base_z + tower_h - wall_t * 0.5)),
        material=shell,
        name="roof_cap",
    )

    outlet_frame_mesh = mesh_from_geometry(
        _build_outlet_frame_mesh(
            outer_width=frame_outer_w,
            outer_height=frame_outer_h,
            opening_width=opening_w,
            opening_height=opening_h,
            depth=frame_depth,
        ),
        "outlet_frame",
    )
    tower.visual(
        outlet_frame_mesh,
        origin=Origin(
            xyz=(
                tower_d * 0.5 + frame_depth * 0.5 - 0.001,
                0.0,
                opening_center_z,
            )
        ),
        material=frame_finish,
        name="outlet_frame",
    )

    tower.inertial = Inertial.from_geometry(
        Box((0.500, 0.400, flange_t + curb_h + tower_h)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, (flange_t + curb_h + tower_h) * 0.5)),
    )

    flap = model.part("weather_flap")
    flap_w = frame_outer_w - 0.010
    flap_h = frame_outer_h - 0.010
    flap_t = 0.012
    hinge_r = 0.006
    flap.visual(
        Box((flap_t, flap_w, flap_h)),
        origin=Origin(
            xyz=(hinge_r + flap_t * 0.35, 0.0, -flap_h * 0.5 - 0.003),
        ),
        material=flap_finish,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=hinge_r, length=flap_w - 0.030),
        origin=Origin(
            xyz=(hinge_r, 0.0, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=hinge_finish,
        name="hinge_barrel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_t, flap_w, flap_h)),
        mass=4.8,
        origin=Origin(xyz=(hinge_r + flap_t * 0.35, 0.0, -flap_h * 0.5 - 0.003)),
    )

    model.articulation(
        "tower_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(
            xyz=(
                tower_d * 0.5 + frame_depth - 0.001,
                0.0,
                opening_center_z + frame_outer_h * 0.5,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.8,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower_housing")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("tower_to_weather_flap")

    closed_panel_aabb = None
    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(
            flap,
            tower,
            axes="yz",
            inner_elem="flap_panel",
            outer_elem="outlet_frame",
            margin=0.004,
            name="closed flap fits within frame footprint",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="x",
            positive_elem="flap_panel",
            negative_elem="outlet_frame",
            min_gap=0.003,
            max_gap=0.012,
            name="closed flap sits just ahead of frame",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    open_panel_aabb = None
    with ctx.pose({hinge: 1.05}):
        open_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    closed_ok = closed_panel_aabb is not None
    open_ok = open_panel_aabb is not None
    outward = (
        closed_ok
        and open_ok
        and open_panel_aabb[1][0] > closed_panel_aabb[1][0] + 0.12
    )
    lifted = (
        closed_ok
        and open_ok
        and open_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.05
    )
    ctx.check(
        "flap opens outward and upward",
        outward and lifted,
        details=(
            f"closed_panel_aabb={closed_panel_aabb}, "
            f"open_panel_aabb={open_panel_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
