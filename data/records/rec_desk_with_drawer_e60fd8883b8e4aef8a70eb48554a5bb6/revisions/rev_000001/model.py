from __future__ import annotations

from dataclasses import dataclass

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DESK_W = 1.80
DESK_D = 0.96
DESK_H = 0.77
TOP_T = 0.04
TOP_OVERHANG = 0.05

CARCASS_H = DESK_H - TOP_T
PED_W = 0.40
PED_D = 0.86
PED_HALF_D = PED_D / 2.0
PED_X = DESK_W / 2.0 - TOP_OVERHANG - PED_W / 2.0

SHELL_T = 0.018
DIVIDER_T = 0.018
SHELF_T = 0.014
PLINTH_H = 0.05
CONNECT = 0.002

DRAWER_FRONT_T = 0.021
DRAWER_PROUD = 0.003
DRAWER_SIDE_T = 0.012
DRAWER_BOTTOM_T = 0.010
DRAWER_BACK_T = 0.010
HANDLE_T = 0.010
HANDLE_H = 0.016
HANDLE_EMBED = 0.003

PED_DRAWER_FRONT_W = 0.368
PED_DRAWER_BODY_W = 0.34
PED_DRAWER_BODY_D = 0.34

CENTER_HOUSING_W = 0.64
CENTER_HOUSING_BOTTOM_Z = 0.552
CENTER_DRAWER_FRONT_W = 0.60
CENTER_DRAWER_BODY_W = 0.58
CENTER_DRAWER_BODY_D = 0.32

BOTTOM_OPEN_Z = 0.058
BOTTOM_FRONT_H = 0.270
MID_OPEN_Z = 0.342
MID_FRONT_H = 0.180
TOP_OPEN_Z = 0.536
TOP_FRONT_H = 0.160


@dataclass(frozen=True)
class DrawerSpec:
    part_name: str
    joint_name: str
    origin_xyz: tuple[float, float, float]
    axis: tuple[float, float, float]
    front_size: tuple[float, float, float]
    body_size: tuple[float, float, float]
    travel: float
    handle_width: float
    retained_overlap: float


def _box_center(bottom_z: float, height: float) -> float:
    return bottom_z + height / 2.0


PEDESTAL_LEVELS = (
    ("bottom", _box_center(BOTTOM_OPEN_Z, BOTTOM_FRONT_H), BOTTOM_FRONT_H, 0.220, 0.22, 0.23),
    ("mid", _box_center(MID_OPEN_Z, MID_FRONT_H), MID_FRONT_H, 0.140, 0.22, 0.18),
    ("top", _box_center(TOP_OPEN_Z, TOP_FRONT_H), TOP_FRONT_H, 0.120, 0.20, 0.17),
)


def build_drawer_specs() -> tuple[DrawerSpec, ...]:
    specs: list[DrawerSpec] = []

    for side_name, x_pos in (("left", -PED_X), ("right", PED_X)):
        for face_name, direction in (("front", -1.0), ("rear", 1.0)):
            for level_name, z_pos, front_h, body_h, travel, handle_w in PEDESTAL_LEVELS:
                specs.append(
                    DrawerSpec(
                        part_name=f"{side_name}_{face_name}_{level_name}_drawer",
                        joint_name=f"{side_name}_{face_name}_{level_name}_slide",
                        origin_xyz=(x_pos, direction * PED_HALF_D, z_pos),
                        axis=(0.0, direction, 0.0),
                        front_size=(PED_DRAWER_FRONT_W, DRAWER_FRONT_T, front_h),
                        body_size=(PED_DRAWER_BODY_W, PED_DRAWER_BODY_D, body_h),
                        travel=travel,
                        handle_width=handle_w,
                        retained_overlap=PED_DRAWER_BODY_D - travel - 0.02,
                    )
                )

    center_z = 0.625
    center_front_h = 0.126
    center_body_h = 0.090
    center_travel = 0.18
    center_handle_w = 0.20
    for face_name, direction in (("front", -1.0), ("rear", 1.0)):
        specs.append(
            DrawerSpec(
                part_name=f"center_{face_name}_drawer",
                joint_name=f"center_{face_name}_slide",
                origin_xyz=(0.0, direction * PED_HALF_D, center_z),
                axis=(0.0, direction, 0.0),
                front_size=(CENTER_DRAWER_FRONT_W, DRAWER_FRONT_T, center_front_h),
                body_size=(CENTER_DRAWER_BODY_W, CENTER_DRAWER_BODY_D, center_body_h),
                travel=center_travel,
                handle_width=center_handle_w,
                retained_overlap=CENTER_DRAWER_BODY_D - center_travel - 0.02,
            )
        )

    return tuple(specs)


DRAWER_SPECS = build_drawer_specs()


def add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def add_pedestal_shell(part, *, side_name: str, x_center: float, wood, shadow) -> None:
    x_sign = 1.0 if x_center > 0.0 else -1.0
    vertical_h = CARCASS_H + CONNECT
    vertical_z = vertical_h / 2.0
    edge_offset = PED_W / 2.0 - SHELL_T / 2.0
    divider_h = CARCASS_H - PLINTH_H - SHELL_T + 2.0 * CONNECT
    divider_z = PLINTH_H + divider_h / 2.0 - CONNECT
    half_compartment_d = PED_HALF_D - DIVIDER_T / 2.0 + CONNECT
    compartment_y = (PED_HALF_D + DIVIDER_T / 2.0) / 2.0

    add_box(
        part,
        name=f"{side_name}_outer_side",
        size=(SHELL_T, PED_D, vertical_h),
        xyz=(x_center + x_sign * edge_offset, 0.0, vertical_z),
        material=wood,
    )
    add_box(
        part,
        name=f"{side_name}_inner_side",
        size=(SHELL_T, PED_D, vertical_h),
        xyz=(x_center - x_sign * edge_offset, 0.0, vertical_z),
        material=wood,
    )
    add_box(
        part,
        name=f"{side_name}_plinth",
        size=(PED_W, PED_D, PLINTH_H + CONNECT),
        xyz=(x_center, 0.0, (PLINTH_H + CONNECT) / 2.0),
        material=shadow,
    )
    add_box(
        part,
        name=f"{side_name}_ceiling",
        size=(PED_W, PED_D, SHELL_T + CONNECT),
        xyz=(x_center, 0.0, CARCASS_H - SHELL_T / 2.0),
        material=wood,
    )
    add_box(
        part,
        name=f"{side_name}_divider",
        size=(PED_W - 2.0 * SHELL_T + CONNECT, DIVIDER_T + CONNECT, divider_h),
        xyz=(x_center, 0.0, divider_z),
        material=shadow,
    )

    shelf_zs = (
        BOTTOM_OPEN_Z + BOTTOM_FRONT_H + SHELF_T / 2.0,
        MID_OPEN_Z + MID_FRONT_H + SHELF_T / 2.0,
    )
    for face_name, direction in (("front", -1.0), ("rear", 1.0)):
        for shelf_index, shelf_z in enumerate(shelf_zs):
            add_box(
                part,
                name=f"{side_name}_{face_name}_shelf_{shelf_index}",
                size=(PED_W - 2.0 * SHELL_T + CONNECT, half_compartment_d, SHELF_T + CONNECT),
                xyz=(x_center, direction * compartment_y, shelf_z),
                material=shadow,
            )


def add_center_drawer_housing(part, *, wood, shadow) -> None:
    wall_h = CARCASS_H - CENTER_HOUSING_BOTTOM_Z + CONNECT
    wall_z = CENTER_HOUSING_BOTTOM_Z + wall_h / 2.0
    side_x = CENTER_HOUSING_W / 2.0 - SHELL_T / 2.0

    add_box(
        part,
        name="center_housing_bottom",
        size=(CENTER_HOUSING_W, PED_D, SHELL_T + CONNECT),
        xyz=(0.0, 0.0, CENTER_HOUSING_BOTTOM_Z + (SHELL_T + CONNECT) / 2.0),
        material=shadow,
    )
    add_box(
        part,
        name="center_left_wall",
        size=(SHELL_T, PED_D, wall_h),
        xyz=(-side_x, 0.0, wall_z),
        material=wood,
    )
    add_box(
        part,
        name="center_right_wall",
        size=(SHELL_T, PED_D, wall_h),
        xyz=(side_x, 0.0, wall_z),
        material=wood,
    )
    add_box(
        part,
        name="center_divider",
        size=(CENTER_HOUSING_W + CONNECT, DIVIDER_T + CONNECT, wall_h),
        xyz=(0.0, 0.0, wall_z),
        material=shadow,
    )


def add_runner_pair(part, spec: DrawerSpec, *, material) -> None:
    body_w, body_d, body_h = spec.body_size
    housing_w = CENTER_HOUSING_W if spec.part_name.startswith("center_") else PED_W
    interior_half_w = (housing_w - 2.0 * SHELL_T) / 2.0
    runner_t = max(0.010, interior_half_w - body_w / 2.0)
    runner_l = max(0.22, body_d - 0.02)
    runner_h = min(0.020, max(0.014, body_h * 0.18))
    x_offset = interior_half_w - runner_t / 2.0
    y_center = spec.axis[1] * (PED_HALF_D - runner_l / 2.0 - 0.004)

    for runner_name, x_sign in (("outer_runner", -1.0), ("inner_runner", 1.0)):
        add_box(
            part,
            name=f"{spec.part_name}_{runner_name}",
            size=(runner_t, runner_l, runner_h),
            xyz=(spec.origin_xyz[0] + x_sign * x_offset, y_center, spec.origin_xyz[2]),
            material=material,
        )


def add_drawer(model: ArticulatedObject, spec: DrawerSpec, *, front_mat, box_mat, hardware_mat) -> None:
    drawer = model.part(spec.part_name)

    outward = spec.axis[1]
    front_w, front_t, front_h = spec.front_size
    body_w, body_d, body_h = spec.body_size
    side_y = -outward * (body_d / 2.0)
    back_y = -outward * (body_d - DRAWER_BACK_T / 2.0)
    front_y = outward * (front_t / 2.0 + CONNECT)
    handle_y = outward * (front_t + DRAWER_PROUD + HANDLE_T / 2.0 - HANDLE_EMBED)
    side_x = body_w / 2.0 - DRAWER_SIDE_T / 2.0
    bottom_z = -body_h / 2.0 + DRAWER_BOTTOM_T / 2.0
    handle_z = min(front_h * 0.18, 0.022)

    add_box(
        drawer,
        name="front",
        size=spec.front_size,
        xyz=(0.0, front_y, 0.0),
        material=front_mat,
    )
    add_box(
        drawer,
        name="left_side",
        size=(DRAWER_SIDE_T, body_d + 2.0 * CONNECT, body_h),
        xyz=(-side_x, side_y, 0.0),
        material=box_mat,
    )
    add_box(
        drawer,
        name="right_side",
        size=(DRAWER_SIDE_T, body_d + 2.0 * CONNECT, body_h),
        xyz=(side_x, side_y, 0.0),
        material=box_mat,
    )
    add_box(
        drawer,
        name="bottom",
        size=(body_w, body_d + 2.0 * CONNECT, DRAWER_BOTTOM_T),
        xyz=(0.0, side_y, bottom_z),
        material=box_mat,
    )
    add_box(
        drawer,
        name="back",
        size=(body_w, DRAWER_BACK_T, body_h),
        xyz=(0.0, back_y, 0.0),
        material=box_mat,
    )
    add_box(
        drawer,
        name="pull",
        size=(spec.handle_width, HANDLE_T, HANDLE_H),
        xyz=(0.0, handle_y, handle_z),
        material=hardware_mat,
    )

    model.articulation(
        spec.joint_name,
        ArticulationType.PRISMATIC,
        parent="desk",
        child=drawer,
        origin=Origin(xyz=spec.origin_xyz),
        axis=spec.axis,
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=spec.travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="partner_desk")

    top_wood = model.material("top_wood", rgba=(0.30, 0.18, 0.10, 1.0))
    body_wood = model.material("body_wood", rgba=(0.36, 0.23, 0.14, 1.0))
    shadow_wood = model.material("shadow_wood", rgba=(0.23, 0.15, 0.09, 1.0))
    drawer_box = model.material("drawer_box", rgba=(0.27, 0.20, 0.15, 1.0))
    hardware = model.material("hardware", rgba=(0.16, 0.15, 0.14, 1.0))

    desk = model.part("desk")
    add_box(
        desk,
        name="top",
        size=(DESK_W, DESK_D, TOP_T),
        xyz=(0.0, 0.0, CARCASS_H + TOP_T / 2.0 - CONNECT / 2.0),
        material=top_wood,
    )

    add_pedestal_shell(desk, side_name="left", x_center=-PED_X, wood=body_wood, shadow=shadow_wood)
    add_pedestal_shell(desk, side_name="right", x_center=PED_X, wood=body_wood, shadow=shadow_wood)
    add_center_drawer_housing(desk, wood=body_wood, shadow=shadow_wood)
    for drawer_spec in DRAWER_SPECS:
        add_runner_pair(desk, drawer_spec, material=shadow_wood)

    for drawer_spec in DRAWER_SPECS:
        add_drawer(model, drawer_spec, front_mat=body_wood, box_mat=drawer_box, hardware_mat=hardware)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk = object_model.get_part("desk")

    for spec in DRAWER_SPECS:
        drawer = object_model.get_part(spec.part_name)
        joint = object_model.get_articulation(spec.joint_name)
        rest_pos = ctx.part_world_position(drawer)

        with ctx.pose({joint: spec.travel}):
            extended_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                desk,
                axes="y",
                elem_a="left_side",
                min_overlap=spec.retained_overlap,
                name=f"{spec.part_name} retains insertion",
            )

        moved_y = None
        drift_x = None
        drift_z = None
        if rest_pos is not None and extended_pos is not None:
            moved_y = extended_pos[1] - rest_pos[1]
            drift_x = extended_pos[0] - rest_pos[0]
            drift_z = extended_pos[2] - rest_pos[2]

        ctx.check(
            f"{spec.part_name} slides straight",
            rest_pos is not None
            and extended_pos is not None
            and abs(moved_y - spec.axis[1] * spec.travel) <= 0.001
            and abs(drift_x) <= 1e-6
            and abs(drift_z) <= 1e-6,
            details=(
                f"rest={rest_pos}, extended={extended_pos}, "
                f"expected_delta_y={spec.axis[1] * spec.travel}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
