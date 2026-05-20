from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _rod_between(part, p0, p1, radius: float, material: Material, name: str) -> None:
    """Add a cylindrical tube whose local +Z axis runs from p0 to p1."""

    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length rod {name}")

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_drying_rack")

    satin_steel = model.material("satin_steel", rgba=(0.78, 0.80, 0.79, 1.0))
    white_coating = model.material("white_powder_coat", rgba=(0.93, 0.94, 0.92, 1.0))
    blue_plastic = model.material("blue_plastic_caps", rgba=(0.08, 0.30, 0.72, 1.0))
    dark_rubber = model.material("dark_rubber_feet", rgba=(0.02, 0.022, 0.024, 1.0))

    length = 1.20
    width = 0.56
    side_y = width * 0.5
    rack_z = 0.82
    hinge_z = 0.77

    top = model.part("top_rack")

    # Perimeter of the light, coated wire drying surface.
    _rod_between(top, (-length / 2, -side_y, rack_z), (length / 2, -side_y, rack_z), 0.012, white_coating, "side_rail_0")
    _rod_between(top, (-length / 2, side_y, rack_z), (length / 2, side_y, rack_z), 0.012, white_coating, "side_rail_1")
    _rod_between(top, (-length / 2, -side_y, rack_z), (-length / 2, side_y, rack_z), 0.012, white_coating, "end_rail_0")
    _rod_between(top, (length / 2, -side_y, rack_z), (length / 2, side_y, rack_z), 0.012, white_coating, "end_rail_1")

    # Many fine horizontal drying bars, welded into the side rails.
    bar_count = 11
    for i in range(bar_count):
        x = -0.48 + i * (0.96 / (bar_count - 1))
        _rod_between(top, (x, -side_y, rack_z + 0.012), (x, side_y, rack_z + 0.012), 0.006, satin_steel, f"drying_bar_{i}")

    # Slightly thicker towel bars at the ends and a central spine to strengthen the grate.
    _rod_between(top, (-0.54, -0.245, rack_z + 0.028), (-0.54, 0.245, rack_z + 0.028), 0.008, white_coating, "end_towel_bar_0")
    _rod_between(top, (0.54, -0.245, rack_z + 0.028), (0.54, 0.245, rack_z + 0.028), 0.008, white_coating, "end_towel_bar_1")
    _rod_between(top, (-length / 2, 0.0, rack_z + 0.004), (length / 2, 0.0, rack_z + 0.004), 0.006, satin_steel, "center_spine")
    for post_index, (x, y) in enumerate(((-0.54, -side_y), (-0.54, side_y), (0.54, -side_y), (0.54, side_y))):
        _rod_between(top, (x, y, rack_z + 0.004), (x, 0.245 if y > 0.0 else -0.245, rack_z + 0.028), 0.006, white_coating, f"towel_bar_post_{post_index}")

    # Plastic corner caps and hinge clevis ears make the rack read as a household product.
    for i, (x, y) in enumerate(
        (
            (-length / 2, -side_y),
            (-length / 2, side_y),
            (length / 2, -side_y),
            (length / 2, side_y),
        )
    ):
        top.visual(Sphere(radius=0.022), origin=Origin(xyz=(x, y, rack_z)), material=blue_plastic, name=f"corner_cap_{i}")

    for side_index, y in enumerate((-side_y, side_y)):
        # Slender steel hinge pin captured by collars on the folding support frame.
        _rod_between(top, (-0.56, y, hinge_z), (0.56, y, hinge_z), 0.006, satin_steel, f"hinge_pin_{side_index}")
        for x in (-0.57, -0.43, 0.43, 0.57):
            top.visual(
                Box((0.026, 0.030, 0.078)),
                origin=Origin(xyz=(x, y, hinge_z + 0.030)),
                material=blue_plastic,
                name=f"hinge_ear_{side_index}_{x:+.2f}",
            )
        # Small latch pockets under the rim, clear of the moving legs.
        top.visual(
            Box((0.075, 0.024, 0.024)),
            origin=Origin(xyz=(0.0, y, hinge_z + 0.006)),
            material=blue_plastic,
            name=f"fold_latch_{side_index}",
        )

    def add_support_frame(name: str, parent_y: float, outward_sign: float, axis) -> None:
        frame = model.part(name)

        y_bottom = outward_sign * 0.18
        y_top = outward_sign * 0.012
        z_top = -0.014
        z_bottom = -0.74
        leg_xs = (-0.50, 0.50)

        for idx, x in enumerate(leg_xs):
            _rod_between(frame, (x - 0.035, 0.0, 0.0), (x + 0.035, 0.0, 0.0), 0.014, white_coating, f"hinge_collar_{idx}")
            frame.visual(Sphere(radius=0.010), origin=Origin(xyz=(x, y_top, -0.010)), material=white_coating, name=f"hinge_node_{idx}")
            _rod_between(frame, (x, y_top, z_top), (x, y_bottom, z_bottom), 0.009, white_coating, f"side_leg_{idx}")
            frame.visual(
                Sphere(radius=0.024),
                origin=Origin(xyz=(x, y_bottom, z_bottom)),
                material=dark_rubber,
                name=f"rubber_foot_{idx}",
            )

        _rod_between(frame, (-0.55, y_bottom, z_bottom), (0.55, y_bottom, z_bottom), 0.012, white_coating, "foot_bar")
        _rod_between(frame, (-0.50, outward_sign * 0.105, -0.43), (0.50, outward_sign * 0.105, -0.43), 0.007, satin_steel, "stiffener_bar")
        _rod_between(frame, (-0.50, outward_sign * 0.070, -0.24), (0.50, outward_sign * 0.070, -0.24), 0.006, satin_steel, "lock_bar")

        model.articulation(
            f"top_to_{name}",
            ArticulationType.REVOLUTE,
            parent=top,
            child=frame,
            origin=Origin(xyz=(0.0, parent_y, hinge_z)),
            axis=axis,
            motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.10),
        )

    add_support_frame("support_frame_0", side_y, 1.0, (-1.0, 0.0, 0.0))
    add_support_frame("support_frame_1", -side_y, -1.0, (1.0, 0.0, 0.0))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    top = object_model.get_part("top_rack")
    frame_0 = object_model.get_part("support_frame_0")
    frame_1 = object_model.get_part("support_frame_1")
    hinge_0 = object_model.get_articulation("top_to_support_frame_0")
    hinge_1 = object_model.get_articulation("top_to_support_frame_1")

    for frame_name, pin_name in (("support_frame_0", "hinge_pin_1"), ("support_frame_1", "hinge_pin_0")):
        for collar_name in ("hinge_collar_0", "hinge_collar_1"):
            ctx.allow_overlap(
                "top_rack",
                frame_name,
                elem_a=pin_name,
                elem_b=collar_name,
                reason="The small steel hinge pin is intentionally captured inside the folding frame collar.",
            )
            ctx.expect_within(
                "top_rack",
                frame_name,
                axes="yz",
                inner_elem=pin_name,
                outer_elem=collar_name,
                margin=0.002,
                name=f"{frame_name} {collar_name} captures hinge pin radially",
            )
            ctx.expect_overlap(
                "top_rack",
                frame_name,
                axes="x",
                elem_a=pin_name,
                elem_b=collar_name,
                min_overlap=0.045,
                name=f"{frame_name} {collar_name} overlaps hinge pin along axis",
            )

    ctx.expect_overlap(top, frame_0, axes="x", elem_a="hinge_pin_1", elem_b="foot_bar", min_overlap=0.90, name="support frame 0 spans most of rack length")
    ctx.expect_overlap(top, frame_1, axes="x", elem_a="hinge_pin_0", elem_b="foot_bar", min_overlap=0.90, name="support frame 1 spans most of rack length")

    rest_foot_0 = ctx.part_element_world_aabb(frame_0, elem="foot_bar")
    rest_foot_1 = ctx.part_element_world_aabb(frame_1, elem="foot_bar")
    ctx.check(
        "unfolded feet sit near floor",
        rest_foot_0 is not None
        and rest_foot_1 is not None
        and rest_foot_0[0][2] < 0.04
        and rest_foot_1[0][2] < 0.04,
        details=f"frame_0={rest_foot_0}, frame_1={rest_foot_1}",
    )

    with ctx.pose({hinge_0: 1.10, hinge_1: 1.10}):
        folded_foot_0 = ctx.part_element_world_aabb(frame_0, elem="foot_bar")
        folded_foot_1 = ctx.part_element_world_aabb(frame_1, elem="foot_bar")

    ctx.check(
        "folding support frames lift under rack",
        rest_foot_0 is not None
        and rest_foot_1 is not None
        and folded_foot_0 is not None
        and folded_foot_1 is not None
        and folded_foot_0[0][2] > rest_foot_0[0][2] + 0.20
        and folded_foot_1[0][2] > rest_foot_1[0][2] + 0.20,
        details=f"rest=({rest_foot_0}, {rest_foot_1}), folded=({folded_foot_0}, {folded_foot_1})",
    )

    return ctx.report()


object_model = build_object_model()
