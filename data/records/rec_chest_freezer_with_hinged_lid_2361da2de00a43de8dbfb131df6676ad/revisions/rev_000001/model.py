from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    rounded_rect_profile,
)


def _ring_mesh(
    name: str,
    *,
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    height: float,
    outer_radius: float,
    inner_radius: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_size[0], outer_size[1], outer_radius, corner_segments=8),
            [rounded_rect_profile(inner_size[0], inner_size[1], inner_radius, corner_segments=8)],
            height=height,
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    body_w = 0.74
    body_d = 0.60
    body_h = 0.81
    wall_th = 0.055
    inner_w = body_w - 2.0 * wall_th
    inner_d = body_d - 2.0 * wall_th
    floor_th = 0.020
    floor_z0 = 0.100
    lid_w = 0.76
    lid_d = 0.60
    lid_th = 0.078
    lid_top_d = 0.584
    lid_top_center_y = 0.322
    hinge_axis_y = -0.295
    hinge_axis_z = body_h + 0.011
    hinge_radius = 0.014
    hinge_x_positions = (-0.235, 0.235)

    model = ArticulatedObject(name="laboratory_sample_chest_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    lid_white = model.material("lid_white", rgba=(0.96, 0.97, 0.98, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.80, 0.83, 0.86, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.58, 0.61, 0.66, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.13, 0.14, 0.15, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.76, 0.79, 1.0))

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    cabinet.visual(
        _ring_mesh(
            "cabinet_shell",
            outer_size=(body_w, body_d),
            inner_size=(inner_w, inner_d),
            height=body_h,
            outer_radius=0.045,
            inner_radius=0.022,
        ),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((body_w, body_d, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=trim_gray,
        name="base_plate",
    )
    cabinet.visual(
        Box((inner_w + 0.004, inner_d + 0.004, floor_th)),
        origin=Origin(xyz=(0.0, 0.0, floor_z0 + floor_th * 0.5)),
        material=liner_gray,
        name="cavity_floor",
    )
    cabinet.visual(
        _ring_mesh(
            "top_liner_rim",
            outer_size=(inner_w + 0.058, inner_d + 0.048),
            inner_size=(inner_w - 0.010, inner_d - 0.010),
            height=0.014,
            outer_radius=0.020,
            inner_radius=0.016,
        ),
        origin=Origin(xyz=(0.0, 0.0, body_h - 0.014)),
        material=liner_gray,
        name="top_liner_rim",
    )
    cabinet.visual(
        Box((0.13, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, body_d * 0.5 + 0.010, body_h - 0.028)),
        material=steel,
        name="front_strike",
    )
    cabinet.visual(
        Box((0.16, 0.030, 0.070)),
        origin=Origin(xyz=(body_w * 0.28, body_d * 0.5 + 0.013, body_h - 0.135)),
        material=trim_gray,
        name="control_pod",
    )

    for side, x_pos in (("left", hinge_x_positions[0]), ("right", hinge_x_positions[1])):
        cabinet.visual(
            Box((0.074, 0.014, 0.022)),
            origin=Origin(xyz=(x_pos, hinge_axis_y - 0.027, hinge_axis_z - 0.033)),
            material=trim_gray,
            name=f"{side}_hinge_bracket",
        )
        cabinet.visual(
            Box((0.018, 0.020, 0.040)),
            origin=Origin(xyz=(x_pos - 0.019, hinge_axis_y - 0.012, hinge_axis_z - 0.020)),
            material=trim_gray,
            name=f"{side}_hinge_post_a",
        )
        cabinet.visual(
            Box((0.018, 0.020, 0.040)),
            origin=Origin(xyz=(x_pos + 0.019, hinge_axis_y - 0.012, hinge_axis_z - 0.020)),
            material=trim_gray,
            name=f"{side}_hinge_post_b",
        )
        cabinet.visual(
            Cylinder(radius=hinge_radius, length=0.018),
            origin=Origin(
                xyz=(x_pos - 0.019, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"{side}_hinge_barrel_a",
        )
        cabinet.visual(
            Cylinder(radius=hinge_radius, length=0.018),
            origin=Origin(
                xyz=(x_pos + 0.019, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"{side}_hinge_barrel_b",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_top_d, lid_th)),
        mass=16.0,
        origin=Origin(xyz=(0.0, lid_top_center_y, lid_th * 0.5 - 0.002)),
    )
    lid.visual(
        Box((lid_w, lid_top_d, lid_th)),
        origin=Origin(xyz=(0.0, lid_top_center_y, lid_th * 0.5 - 0.002)),
        material=lid_white,
        name="lid_panel",
    )
    lid.visual(
        Box((inner_w + 0.070, inner_d + 0.050, 0.016)),
        origin=Origin(xyz=(0.0, 0.295, 0.008)),
        material=liner_gray,
        name="lid_inner_liner",
    )
    lid.visual(
        _ring_mesh(
            "seal_gasket",
            outer_size=(inner_w + 0.032, inner_d + 0.022),
            inner_size=(inner_w - 0.028, inner_d - 0.038),
            height=0.012,
            outer_radius=0.018,
            inner_radius=0.014,
        ),
        origin=Origin(xyz=(0.0, 0.295, -0.011)),
        material=gasket_black,
        name="seal_gasket",
    )
    lid.visual(
        Box((0.150, 0.020, 0.036)),
        origin=Origin(xyz=(0.0, 0.610, 0.012)),
        material=trim_gray,
        name="latch_mount",
    )
    lid.visual(
        Box((lid_w, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.031, 0.016)),
        material=lid_white,
        name="rear_lid_skirt",
    )

    for side, x_pos in (("left", hinge_x_positions[0]), ("right", hinge_x_positions[1])):
        lid.visual(
            Box((0.060, 0.018, 0.010)),
            origin=Origin(xyz=(x_pos, 0.028, 0.011)),
            material=trim_gray,
            name=f"{side}_hinge_anchor",
        )
        lid.visual(
            Box((0.016, 0.024, 0.022)),
            origin=Origin(xyz=(x_pos, 0.010, 0.007)),
            material=trim_gray,
            name=f"{side}_hinge_leaf",
        )
        lid.visual(
            Cylinder(radius=hinge_radius, length=0.018),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"{side}_lid_knuckle",
        )

    latch = model.part("latch")
    latch.inertial = Inertial.from_geometry(
        Box((0.13, 0.040, 0.12)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.018, -0.045)),
    )
    latch.visual(
        Box((0.122, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, 0.012, -0.055)),
        material=latch_dark,
        name="handle_plate",
    )
    latch.visual(
        Cylinder(radius=0.012, length=0.134),
        origin=Origin(xyz=(0.0, 0.017, -0.098), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="handle_grip",
    )
    latch.visual(
        Cylinder(radius=0.008, length=0.092),
        origin=Origin(xyz=(0.0, 0.015, -0.016), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="cam_lobe",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=0.0, upper=1.28),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, 0.617, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    latch_joint = object_model.get_articulation("lid_to_latch")

    lid.get_visual("left_lid_knuckle")
    lid.get_visual("right_lid_knuckle")
    cabinet.get_visual("front_strike")
    latch.get_visual("cam_lobe")

    with ctx.pose({lid_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="seal_gasket",
            negative_elem="cabinet_shell",
            max_gap=0.003,
            max_penetration=0.002,
            name="lid gasket seats on cabinet rim",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="xy",
            elem_a="lid_panel",
            elem_b="cabinet_shell",
            min_overlap=0.55,
            name="lid covers cabinet opening footprint",
        )
        ctx.expect_gap(
            latch,
            cabinet,
            axis="y",
            positive_elem="cam_lobe",
            negative_elem="front_strike",
            min_gap=0.004,
            max_gap=0.022,
            name="closed cam latch sits just ahead of strike",
        )

    def _elem_max(part_obj, elem_name: str, axis_index: int) -> float | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return aabb[1][axis_index]

    closed_lid_top = _elem_max(lid, "lid_panel", 2)
    closed_mount_top = _elem_max(lid, "latch_mount", 2)
    with ctx.pose({lid_hinge: 1.18}):
        open_lid_top = _elem_max(lid, "lid_panel", 2)
        open_mount_top = _elem_max(lid, "latch_mount", 2)
        ctx.check(
            "lid opens upward on rear hinge axis",
            closed_lid_top is not None and open_lid_top is not None and open_lid_top > closed_lid_top + 0.28,
            details=f"closed_top={closed_lid_top}, open_top={open_lid_top}",
        )
        ctx.check(
            "front edge of lid lifts well above cabinet when open",
            closed_mount_top is not None and open_mount_top is not None and open_mount_top > closed_mount_top + 0.22,
            details=f"closed_mount_top={closed_mount_top}, open_mount_top={open_mount_top}",
        )

    closed_handle_front = _elem_max(latch, "handle_plate", 1)
    with ctx.pose({latch_joint: 1.0}):
        open_handle_front = _elem_max(latch, "handle_plate", 1)
        ctx.check(
            "front latch handle rotates outward",
            closed_handle_front is not None and open_handle_front is not None and open_handle_front > closed_handle_front + 0.02,
            details=f"closed_front={closed_handle_front}, open_front={open_handle_front}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
