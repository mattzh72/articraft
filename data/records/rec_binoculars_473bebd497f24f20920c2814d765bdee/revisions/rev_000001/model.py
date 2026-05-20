from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A softly radiused rectangular housing, authored around its local origin."""

    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    mn, mx = aabb
    return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rangefinder_binocular")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.006, 1.0))
    armor = model.material("charcoal_armor", rgba=(0.025, 0.028, 0.030, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.09, 0.095, 0.10, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.05, 0.18, 0.28, 0.72))
    display = model.material("rangefinder_window", rgba=(0.0, 0.04, 0.05, 0.92))
    white = model.material("white_marking", rgba=(0.92, 0.92, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.009, length=0.150),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_pin",
    )
    body.visual(
        Box((0.012, 0.012, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_metal,
        name="hinge_post",
    )
    body.visual(
        mesh_from_cadquery(
            _rounded_box((0.052, 0.106, 0.022), 0.005),
            "central_rangefinder_bridge",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(0.0, -0.002, 0.060)),
        material=armor,
        name="top_bridge",
    )
    body.visual(
        Box((0.030, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, 0.052, 0.058)),
        material=display,
        name="front_window",
    )
    body.visual(
        Box((0.020, 0.026, 0.003)),
        origin=Origin(xyz=(0.0, -0.022, 0.0725)),
        material=display,
        name="top_readout",
    )
    for x, name in ((-0.025, "focus_yoke_0"), (0.025, "focus_yoke_1")):
        body.visual(
            Box((0.008, 0.032, 0.034)),
            origin=Origin(xyz=(x, -0.014, 0.083)),
            material=dark_metal,
            name=name,
        )

    housing_mesh = _rounded_box((0.068, 0.088, 0.066), 0.009)

    def add_barrel(part_name: str, side: float) -> object:
        barrel = model.part(part_name)
        sx = 0.064 * side
        barrel.visual(
            mesh_from_cadquery(
                housing_mesh,
                f"{part_name}_housing",
                tolerance=0.0008,
            ),
            origin=Origin(xyz=(sx, -0.005, 0.0)),
            material=armor,
            name="housing",
        )
        barrel.visual(
            Cylinder(radius=0.032, length=0.080),
            origin=Origin(xyz=(sx, 0.072, -0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="objective_barrel",
        )
        for i, y in enumerate((0.045, 0.065, 0.087)):
            barrel.visual(
                Cylinder(radius=0.034, length=0.006),
                origin=Origin(xyz=(sx, y, -0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_metal,
                name=f"objective_rib_{i}",
            )
        barrel.visual(
            Cylinder(radius=0.026, length=0.005),
            origin=Origin(xyz=(sx, 0.111, -0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=glass,
            name="front_lens",
        )
        barrel.visual(
            Cylinder(radius=0.022, length=0.036),
            origin=Origin(xyz=(sx, -0.065, 0.003), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="eyepiece",
        )
        barrel.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(sx, -0.085, 0.003), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=glass,
            name="rear_lens",
        )
        y_lug = -0.030 if side < 0 else 0.030
        barrel.visual(
            Box((0.060, 0.045, 0.025)),
            origin=Origin(xyz=(0.024 * side, y_lug, 0.017)),
            material=dark_metal,
            name="hinge_arm",
        )
        barrel.visual(
            Cylinder(radius=0.017, length=0.052),
            origin=Origin(xyz=(0.0, y_lug, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_sleeve",
        )
        barrel.visual(
            Box((0.010, 0.052, 0.026)),
            origin=Origin(xyz=(side * 0.096, -0.010, 0.002)),
            material=matte_black,
            name="palm_grip",
        )
        return barrel

    left_barrel = add_barrel("left_barrel", -1.0)
    right_barrel = add_barrel("right_barrel", 1.0)
    right_barrel.visual(
        Cylinder(radius=0.021, length=0.007),
        origin=Origin(xyz=(0.064, -0.006, 0.0365)),
        material=dark_metal,
        name="selector_boss",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="wheel",
    )
    focus_wheel.visual(
        Cylinder(radius=0.005, length=0.062),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="focus_axle",
    )
    focus_wheel.visual(
        Box((0.026, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=white,
        name="focus_mark",
    )
    for i in range(16):
        angle = i * math.tau / 16.0
        focus_wheel.visual(
            Box((0.035, 0.003, 0.006)),
            origin=Origin(
                xyz=(0.0, 0.019 * math.sin(angle), 0.019 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"grip_ridge_{i}",
        )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.018, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=dark_metal,
        name="dial_cap",
    )
    selector_dial.visual(
        Box((0.015, 0.004, 0.003)),
        origin=Origin(xyz=(0.0065, 0.0, 0.010)),
        material=white,
        name="dial_pointer",
    )

    model.articulation(
        "body_to_left_barrel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_barrel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.20, upper=0.20),
    )
    model.articulation(
        "body_to_right_barrel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_barrel,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.20, upper=0.20),
    )
    model.articulation(
        "body_to_focus_wheel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=focus_wheel,
        origin=Origin(xyz=(0.0, -0.014, 0.092)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "right_barrel_to_selector",
        ArticulationType.REVOLUTE,
        parent=right_barrel,
        child=selector_dial,
        origin=Origin(xyz=(0.064, -0.006, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=3.0, lower=0.0, upper=2.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    focus_wheel = object_model.get_part("focus_wheel")
    selector_dial = object_model.get_part("selector_dial")
    left_hinge = object_model.get_articulation("body_to_left_barrel")
    right_hinge = object_model.get_articulation("body_to_right_barrel")
    focus_joint = object_model.get_articulation("body_to_focus_wheel")
    selector_joint = object_model.get_articulation("right_barrel_to_selector")

    ctx.allow_overlap(
        body,
        left_barrel,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The central hinge pin is intentionally captured inside the left interpupillary sleeve proxy.",
    )
    ctx.allow_overlap(
        body,
        right_barrel,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The central hinge pin is intentionally captured inside the right interpupillary sleeve proxy.",
    )
    for yoke in ("focus_yoke_0", "focus_yoke_1"):
        ctx.allow_overlap(
            body,
            focus_wheel,
            elem_a=yoke,
            elem_b="focus_axle",
            reason="The focus wheel axle is intentionally seated through the fixed yoke bracket proxy.",
        )

    ctx.expect_within(
        body,
        left_barrel,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_sleeve",
        margin=0.001,
        name="left hinge sleeve surrounds central pin",
    )
    ctx.expect_overlap(
        body,
        left_barrel,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.045,
        name="left sleeve has retained hinge length",
    )
    ctx.expect_within(
        body,
        right_barrel,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_sleeve",
        margin=0.001,
        name="right hinge sleeve surrounds central pin",
    )
    ctx.expect_overlap(
        body,
        right_barrel,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.045,
        name="right sleeve has retained hinge length",
    )
    for yoke in ("focus_yoke_0", "focus_yoke_1"):
        ctx.expect_within(
            focus_wheel,
            body,
            axes="yz",
            inner_elem="focus_axle",
            outer_elem=yoke,
            margin=0.001,
            name=f"focus axle is centered in {yoke}",
        )
        ctx.expect_overlap(
            focus_wheel,
            body,
            axes="x",
            elem_a="focus_axle",
            elem_b=yoke,
            min_overlap=0.006,
            name=f"focus axle passes through {yoke}",
        )
    ctx.expect_contact(
        selector_dial,
        right_barrel,
        elem_a="dial_cap",
        elem_b="selector_boss",
        contact_tol=0.002,
        name="selector dial seats on right barrel boss",
    )

    rest_right = _center_from_aabb(ctx.part_element_world_aabb(right_barrel, elem="housing"))
    with ctx.pose({right_hinge: 0.16}):
        angled_right = _center_from_aabb(ctx.part_element_world_aabb(right_barrel, elem="housing"))
    ctx.check(
        "right barrel pivots around central interpupillary hinge",
        rest_right is not None
        and angled_right is not None
        and abs(angled_right[2] - rest_right[2]) > 0.007,
        details=f"rest={rest_right}, angled={angled_right}",
    )

    rest_left = _center_from_aabb(ctx.part_element_world_aabb(left_barrel, elem="housing"))
    with ctx.pose({left_hinge: -0.16}):
        angled_left = _center_from_aabb(ctx.part_element_world_aabb(left_barrel, elem="housing"))
    ctx.check(
        "left barrel pivots around central interpupillary hinge",
        rest_left is not None
        and angled_left is not None
        and abs(angled_left[2] - rest_left[2]) > 0.007,
        details=f"rest={rest_left}, angled={angled_left}",
    )

    focus_rest = _center_from_aabb(ctx.part_element_world_aabb(focus_wheel, elem="focus_mark"))
    with ctx.pose({focus_joint: 0.85}):
        focus_turned = _center_from_aabb(ctx.part_element_world_aabb(focus_wheel, elem="focus_mark"))
    ctx.check(
        "center focus wheel marker rotates",
        focus_rest is not None
        and focus_turned is not None
        and abs(focus_turned[1] - focus_rest[1]) > 0.010,
        details=f"rest={focus_rest}, turned={focus_turned}",
    )

    dial_rest = _center_from_aabb(ctx.part_element_world_aabb(selector_dial, elem="dial_pointer"))
    with ctx.pose({selector_joint: 1.1}):
        dial_turned = _center_from_aabb(ctx.part_element_world_aabb(selector_dial, elem="dial_pointer"))
    ctx.check(
        "ranging mode selector pointer rotates on right barrel",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_turned[1] - dial_rest[1]) > 0.004,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    return ctx.report()


object_model = build_object_model()
