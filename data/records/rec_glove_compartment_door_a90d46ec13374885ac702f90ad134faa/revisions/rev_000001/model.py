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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aircraft_glove_compartment")

    panel_gray = model.material("panel_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    door_gray = model.material("door_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.76, 0.77, 0.79, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.30, 0.31, 0.33, 1.0))

    panel_width = 0.48
    panel_height = 0.32
    panel_thickness = 0.018
    opening_width = 0.326
    opening_height = 0.206
    door_width = 0.318
    door_height = 0.200
    door_thickness = 0.010
    wall_thickness = 0.010
    jamb_depth = 0.009
    cavity_depth = 0.220
    hinge_axis_x = 0.010
    hinge_axis_y = -0.168
    hinge_barrel_radius = 0.005
    hinge_barrel_length = 0.040
    hinge_z_offset = 0.065

    side_margin = 0.5 * (panel_width - opening_width)
    top_margin = 0.5 * (panel_height - opening_height)
    opening_half_w = 0.5 * opening_width
    opening_half_h = 0.5 * opening_height
    cavity_inner_w = opening_width - 2.0 * wall_thickness
    cavity_inner_h = opening_height - 2.0 * wall_thickness

    panel_box = model.part("panel_box")
    panel_box.visual(
        Box((panel_thickness, side_margin, panel_height)),
        origin=Origin(xyz=(-0.5 * panel_thickness, -(opening_half_w + 0.5 * side_margin), 0.0)),
        material=panel_gray,
        name="frame_left",
    )
    panel_box.visual(
        Box((panel_thickness, side_margin, panel_height)),
        origin=Origin(xyz=(-0.5 * panel_thickness, opening_half_w + 0.5 * side_margin, 0.0)),
        material=panel_gray,
        name="frame_right",
    )
    panel_box.visual(
        Box((panel_thickness, opening_width, top_margin)),
        origin=Origin(xyz=(-0.5 * panel_thickness, 0.0, opening_half_h + 0.5 * top_margin)),
        material=panel_gray,
        name="frame_top",
    )
    panel_box.visual(
        Box((panel_thickness, opening_width, top_margin)),
        origin=Origin(xyz=(-0.5 * panel_thickness, 0.0, -(opening_half_h + 0.5 * top_margin))),
        material=panel_gray,
        name="frame_bottom",
    )

    panel_box.visual(
        Box((jamb_depth, wall_thickness, opening_height)),
        origin=Origin(xyz=(-panel_thickness + 0.5 * jamb_depth, -(opening_half_w - 0.5 * wall_thickness), 0.0)),
        material=cavity_dark,
        name="jamb_left",
    )
    panel_box.visual(
        Box((jamb_depth, wall_thickness, opening_height)),
        origin=Origin(xyz=(-panel_thickness + 0.5 * jamb_depth, opening_half_w - 0.5 * wall_thickness, 0.0)),
        material=cavity_dark,
        name="jamb_right",
    )
    panel_box.visual(
        Box((jamb_depth, cavity_inner_w, wall_thickness)),
        origin=Origin(xyz=(-panel_thickness + 0.5 * jamb_depth, 0.0, opening_half_h - 0.5 * wall_thickness)),
        material=cavity_dark,
        name="jamb_top",
    )
    panel_box.visual(
        Box((jamb_depth, cavity_inner_w, wall_thickness)),
        origin=Origin(xyz=(-panel_thickness + 0.5 * jamb_depth, 0.0, -(opening_half_h - 0.5 * wall_thickness))),
        material=cavity_dark,
        name="jamb_bottom",
    )

    panel_box.visual(
        Box((cavity_depth, wall_thickness, opening_height)),
        origin=Origin(
            xyz=(
                -panel_thickness - 0.5 * cavity_depth,
                -(opening_half_w - 0.5 * wall_thickness),
                0.0,
            )
        ),
        material=cavity_dark,
        name="box_side_left",
    )
    panel_box.visual(
        Box((cavity_depth, wall_thickness, opening_height)),
        origin=Origin(
            xyz=(
                -panel_thickness - 0.5 * cavity_depth,
                opening_half_w - 0.5 * wall_thickness,
                0.0,
            )
        ),
        material=cavity_dark,
        name="box_side_right",
    )
    panel_box.visual(
        Box((cavity_depth, cavity_inner_w, wall_thickness)),
        origin=Origin(
            xyz=(
                -panel_thickness - 0.5 * cavity_depth,
                0.0,
                opening_half_h - 0.5 * wall_thickness,
            )
        ),
        material=cavity_dark,
        name="box_top",
    )
    panel_box.visual(
        Box((cavity_depth, cavity_inner_w, wall_thickness)),
        origin=Origin(
            xyz=(
                -panel_thickness - 0.5 * cavity_depth,
                0.0,
                -(opening_half_h - 0.5 * wall_thickness),
            )
        ),
        material=cavity_dark,
        name="box_bottom",
    )
    panel_box.visual(
        Box((wall_thickness, cavity_inner_w, cavity_inner_h)),
        origin=Origin(
            xyz=(
                -panel_thickness - cavity_depth + 0.5 * wall_thickness,
                0.0,
                0.0,
            )
        ),
        material=cavity_dark,
        name="box_back",
    )

    for suffix, z_center in (("upper", hinge_z_offset), ("lower", -hinge_z_offset)):
        panel_box.visual(
            Box((0.008, 0.030, 0.054)),
            origin=Origin(xyz=(0.0025, hinge_axis_y - 0.015, z_center)),
            material=hinge_metal,
            name=f"hinge_plate_{suffix}",
        )
        panel_box.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, z_center)),
            material=hinge_metal,
            name=f"hinge_barrel_{suffix}",
        )

    door = model.part("door")
    door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(-0.013, 0.165, 0.0)),
        material=door_gray,
        name="door_skin",
    )
    door.visual(
        Box((0.004, 0.028, 0.050)),
        origin=Origin(xyz=(-0.010, 0.017, hinge_z_offset)),
        material=hinge_metal,
        name="hinge_leaf_upper",
    )
    door.visual(
        Box((0.004, 0.028, 0.050)),
        origin=Origin(xyz=(-0.010, 0.017, -hinge_z_offset)),
        material=hinge_metal,
        name="hinge_leaf_lower",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.002),
        origin=Origin(xyz=(-0.007, 0.258, 0.0), rpy=(0.0, 0.5 * pi, 0.0)),
        material=hinge_metal,
        name="latch_bezel",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, 0.5 * pi, 0.0)),
        material=latch_metal,
        name="latch_boss",
    )
    latch.visual(
        Box((0.004, 0.016, 0.055)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=latch_metal,
        name="latch_handle",
    )

    model.articulation(
        "panel_to_door",
        ArticulationType.REVOLUTE,
        parent=panel_box,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(-0.006, 0.258, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=0.5 * pi,
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
    panel_box = object_model.get_part("panel_box")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("panel_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    def _extent(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check("panel box part exists", panel_box is not None)
    ctx.check("door part exists", door is not None)
    ctx.check("latch part exists", latch is not None)

    door_axis_ok = door_hinge.axis == (0.0, 0.0, -1.0)
    latch_axis_ok = latch_joint.axis == (1.0, 0.0, 0.0)
    ctx.check(
        "door hinge uses a vertical side axis",
        door_axis_ok,
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "latch uses a front to back quarter turn axis",
        latch_axis_ok,
        details=f"axis={latch_joint.axis}",
    )

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            door,
            panel_box,
            axis="y",
            positive_elem="door_skin",
            negative_elem="frame_left",
            min_gap=0.001,
            max_gap=0.006,
            name="door hinge edge clears left frame",
        )
        ctx.expect_gap(
            panel_box,
            door,
            axis="y",
            positive_elem="frame_right",
            negative_elem="door_skin",
            min_gap=0.006,
            max_gap=0.012,
            name="door free edge clears right frame",
        )
        ctx.expect_gap(
            panel_box,
            door,
            axis="z",
            positive_elem="frame_top",
            negative_elem="door_skin",
            min_gap=0.002,
            max_gap=0.006,
            name="door top edge clears top frame",
        )
        ctx.expect_gap(
            door,
            panel_box,
            axis="z",
            positive_elem="door_skin",
            negative_elem="frame_bottom",
            min_gap=0.002,
            max_gap=0.006,
            name="door bottom edge clears bottom frame",
        )
        ctx.expect_gap(
            latch,
            door,
            axis="x",
            positive_elem="latch_boss",
            negative_elem="latch_bezel",
            min_gap=0.0,
            max_gap=0.0005,
            name="latch boss seats against bezel without interpenetration",
        )

        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
        closed_handle_aabb = ctx.part_element_world_aabb(latch, elem="latch_handle")

    with ctx.pose({door_hinge: 1.2, latch_joint: 0.0}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_skin")

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.5 * pi}):
        turned_handle_aabb = ctx.part_element_world_aabb(latch, elem="latch_handle")

    closed_max_x = None if closed_door_aabb is None else closed_door_aabb[1][0]
    open_max_x = None if open_door_aabb is None else open_door_aabb[1][0]
    ctx.check(
        "door swings outward when opened",
        closed_max_x is not None and open_max_x is not None and open_max_x > closed_max_x + 0.12,
        details=f"closed_max_x={closed_max_x}, open_max_x={open_max_x}",
    )

    closed_handle_y = _extent(closed_handle_aabb, "y")
    closed_handle_z = _extent(closed_handle_aabb, "z")
    turned_handle_y = _extent(turned_handle_aabb, "y")
    turned_handle_z = _extent(turned_handle_aabb, "z")
    ctx.check(
        "latch handle rotates a quarter turn across the door face",
        (
            closed_handle_y is not None
            and closed_handle_z is not None
            and turned_handle_y is not None
            and turned_handle_z is not None
            and closed_handle_z > closed_handle_y
            and turned_handle_y > turned_handle_z
        ),
        details=(
            f"closed_y={closed_handle_y}, closed_z={closed_handle_z}, "
            f"turned_y={turned_handle_y}, turned_z={turned_handle_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
