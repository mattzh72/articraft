from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_panel(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A lightly radiused refrigerator door slab, authored in meters."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _add_door_leaf(
    model: ArticulatedObject,
    case,
    *,
    name: str,
    hinge_origin: tuple[float, float, float],
    panel_width: float,
    panel_height: float,
    side: str,
    pin_elem: str,
    panel_material: Material,
    trim_material: Material,
    gasket_material: Material,
    handle_material: Material,
):
    """Create one French-door leaf whose local frame is its vertical hinge axis."""
    is_left = side == "left"
    sign = 1.0 if is_left else -1.0
    door_t = 0.055
    collar_radius = 0.016
    hinge_to_panel_edge = 0.024
    panel_center_x = sign * (hinge_to_panel_edge + panel_width * 0.5)
    handle_x = sign * (hinge_to_panel_edge + panel_width - 0.075)
    seam_strip_x = sign * (hinge_to_panel_edge + panel_width - 0.010)

    door = model.part(name)
    door.visual(
        mesh_from_cadquery(
            _rounded_panel((panel_width, door_t, panel_height), 0.018),
            f"{name}_panel",
            tolerance=0.0012,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(panel_center_x, 0.0, panel_height * 0.5)),
        material=panel_material,
        name="door_panel",
    )

    # Black magnetic gasket lines make the four-leaf front read as distinct doors.
    door.visual(
        Box((0.018, 0.004, panel_height * 0.96)),
        origin=Origin(xyz=(seam_strip_x, -door_t * 0.5 - 0.002, panel_height * 0.5)),
        material=gasket_material,
        name="center_gasket",
    )
    door.visual(
        Box((panel_width * 0.92, 0.004, 0.012)),
        origin=Origin(xyz=(panel_center_x, -door_t * 0.5 - 0.002, 0.030)),
        material=gasket_material,
        name="lower_gasket",
    )
    door.visual(
        Box((panel_width * 0.92, 0.004, 0.012)),
        origin=Origin(xyz=(panel_center_x, -door_t * 0.5 - 0.002, panel_height - 0.030)),
        material=gasket_material,
        name="upper_gasket",
    )

    # A continuous door-side hinge collar is clipped to the matching case pin.
    door.visual(
        Cylinder(radius=collar_radius, length=panel_height),
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.5)),
        material=trim_material,
        name="hinge_collar",
    )
    door.visual(
        Box((0.020, 0.018, panel_height * 0.82)),
        origin=Origin(xyz=(sign * 0.022, 0.0, panel_height * 0.5)),
        material=trim_material,
        name="hinge_clip",
    )

    handle_height = 0.72 if panel_height > 0.8 else 0.32
    handle_center_z = panel_height * 0.52
    door.visual(
        Cylinder(radius=0.016, length=handle_height),
        origin=Origin(xyz=(handle_x, -0.080, handle_center_z)),
        material=handle_material,
        name="handle_bar",
    )
    for i, z in enumerate((handle_center_z - handle_height * 0.36, handle_center_z + handle_height * 0.36)):
        door.visual(
            Cylinder(radius=0.010, length=0.052),
            origin=Origin(xyz=(handle_x, -0.053, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=handle_material,
            name=f"handle_standoff_{i}",
        )

    axis = (0.0, 0.0, -1.0) if is_left else (0.0, 0.0, 1.0)
    joint = model.articulation(
        f"case_to_{name}",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=hinge_origin),
        axis=axis,
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=0.0, upper=1.75),
        meta={"pin_elem": pin_elem},
    )
    return door, joint


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_door_french_refrigerator")

    stainless = model.material("warm_brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    side_metal = model.material("soft_silver_case", rgba=(0.62, 0.64, 0.63, 1.0))
    dark = model.material("black_rubber_gaskets", rgba=(0.015, 0.016, 0.017, 1.0))
    shadow = model.material("dark_recess", rgba=(0.055, 0.058, 0.060, 1.0))
    satin = model.material("satin_handle_metal", rgba=(0.88, 0.88, 0.84, 1.0))

    case = model.part("case")
    # Full-depth body behind the doors, plus visible mullions and kick grille.
    case.visual(
        Box((1.28, 0.70, 1.82)),
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        material=side_metal,
        name="main_body",
    )
    case.visual(
        Box((1.18, 0.020, 1.72)),
        origin=Origin(xyz=(0.0, -0.355, 0.93)),
        material=shadow,
        name="front_recess",
    )
    case.visual(
        Box((0.012, 0.050, 1.66)),
        origin=Origin(xyz=(0.0, -0.386, 0.94)),
        material=dark,
        name="center_mullion",
    )
    case.visual(
        Box((1.16, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.386, 0.64)),
        material=dark,
        name="middle_mullion",
    )
    case.visual(
        Box((1.10, 0.018, 0.105)),
        origin=Origin(xyz=(0.0, -0.358, 0.045)),
        material=shadow,
        name="toe_kick_panel",
    )
    for i, z in enumerate((0.020, 0.045, 0.070)):
        case.visual(
            Box((0.92, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, -0.366, z)),
            material=side_metal,
            name=f"kick_grille_slat_{i}",
        )

    hinge_y = -0.397
    left_x = -0.585
    right_x = 0.585
    upper_bottom = 0.665
    upper_height = 1.105
    lower_bottom = 0.105
    lower_height = 0.515
    panel_width = 0.552

    # Case-side hinge barrels/pins: four independent vertical axes, one per leaf.
    hinge_specs = (
        ("upper_left_hinge_pin", left_x, upper_bottom, upper_height),
        ("upper_right_hinge_pin", right_x, upper_bottom, upper_height),
        ("lower_left_hinge_pin", left_x, lower_bottom, lower_height),
        ("lower_right_hinge_pin", right_x, lower_bottom, lower_height),
    )
    for elem, x, bottom, height in hinge_specs:
        case.visual(
            Cylinder(radius=0.008, length=height + 0.035),
            origin=Origin(xyz=(x, hinge_y, bottom + height * 0.5)),
            material=dark,
            name=elem,
        )

    _add_door_leaf(
        model,
        case,
        name="upper_left_door",
        hinge_origin=(left_x, hinge_y, upper_bottom),
        panel_width=panel_width,
        panel_height=upper_height,
        side="left",
        pin_elem="upper_left_hinge_pin",
        panel_material=stainless,
        trim_material=side_metal,
        gasket_material=dark,
        handle_material=satin,
    )
    _add_door_leaf(
        model,
        case,
        name="upper_right_door",
        hinge_origin=(right_x, hinge_y, upper_bottom),
        panel_width=panel_width,
        panel_height=upper_height,
        side="right",
        pin_elem="upper_right_hinge_pin",
        panel_material=stainless,
        trim_material=side_metal,
        gasket_material=dark,
        handle_material=satin,
    )
    _add_door_leaf(
        model,
        case,
        name="lower_left_door",
        hinge_origin=(left_x, hinge_y, lower_bottom),
        panel_width=panel_width,
        panel_height=lower_height,
        side="left",
        pin_elem="lower_left_hinge_pin",
        panel_material=stainless,
        trim_material=side_metal,
        gasket_material=dark,
        handle_material=satin,
    )
    _add_door_leaf(
        model,
        case,
        name="lower_right_door",
        hinge_origin=(right_x, hinge_y, lower_bottom),
        panel_width=panel_width,
        panel_height=lower_height,
        side="right",
        pin_elem="lower_right_hinge_pin",
        panel_material=stainless,
        trim_material=side_metal,
        gasket_material=dark,
        handle_material=satin,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    door_names = (
        "upper_left_door",
        "upper_right_door",
        "lower_left_door",
        "lower_right_door",
    )
    pin_for = {
        "upper_left_door": "upper_left_hinge_pin",
        "upper_right_door": "upper_right_hinge_pin",
        "lower_left_door": "lower_left_hinge_pin",
        "lower_right_door": "lower_right_hinge_pin",
    }

    ctx.check(
        "four independent hinged door leaves",
        all(object_model.get_part(name) is not None for name in door_names)
        and all(object_model.get_articulation(f"case_to_{name}") is not None for name in door_names),
    )

    for name in door_names:
        door = object_model.get_part(name)
        joint = object_model.get_articulation(f"case_to_{name}")
        pin_elem = pin_for[name]
        ctx.allow_overlap(
            case,
            door,
            elem_a=pin_elem,
            elem_b="hinge_collar",
            reason="The modeled solid hinge collar represents a door clip captured around the case-side barrel pin.",
        )
        ctx.expect_within(
            case,
            door,
            axes="xy",
            inner_elem=pin_elem,
            outer_elem="hinge_collar",
            margin=0.001,
            name=f"{name} collar surrounds its hinge barrel",
        )
        ctx.expect_overlap(
            case,
            door,
            axes="z",
            elem_a=pin_elem,
            elem_b="hinge_collar",
            min_overlap=0.45 if "upper" in name else 0.22,
            name=f"{name} remains vertically clipped to barrel",
        )
        ctx.expect_gap(
            case,
            door,
            axis="y",
            positive_elem="main_body",
            negative_elem="door_panel",
            min_gap=0.002,
            max_gap=0.050,
            name=f"{name} door slab is proud of case front",
        )
        ctx.check(
            f"{name} has vertical revolute hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 3) for v in joint.axis) in {(0.0, 0.0, 1.0), (0.0, 0.0, -1.0)}
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper >= 1.5,
        )

    def _elem_center_y(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) * 0.5

    for name in door_names:
        door = object_model.get_part(name)
        joint = object_model.get_articulation(f"case_to_{name}")
        with ctx.pose({joint: 0.0}):
            closed_y = _elem_center_y(door, "handle_bar")
        with ctx.pose({joint: 1.1}):
            open_y = _elem_center_y(door, "handle_bar")
        ctx.check(
            f"{name} swings outward from center seam",
            closed_y is not None and open_y is not None and open_y < closed_y - 0.16,
            details=f"closed handle y={closed_y}, open handle y={open_y}",
        )

    return ctx.report()


object_model = build_object_model()
