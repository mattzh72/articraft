from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _circle_points(cx: float, radius: float, *, segments: int = 32) -> list[tuple[float, float, float]]:
    """Circle in the glasses front plane (X/Z, with Y as thickness/front-back)."""
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            0.0,
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _wire_mesh(points, name: str, *, radius: float, closed: bool = False):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            closed_spline=closed,
            samples_per_segment=10,
            radial_segments=18,
            cap_ends=not closed,
            up_hint=(0.0, 1.0, 0.0),
        ),
        name,
    )


def _pad_mesh(name: str):
    pad = ExtrudeGeometry(
        superellipse_profile(0.006, 0.012, exponent=2.4, segments=36),
        0.0024,
        cap=True,
        center=True,
    )
    return mesh_from_geometry(pad, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_wire_glasses")

    polished_steel = Material("polished_steel", rgba=(0.72, 0.74, 0.75, 1.0))
    pale_lens = Material("pale_lens", rgba=(0.78, 0.92, 1.0, 0.28))
    soft_silicone = Material("soft_silicone", rgba=(0.92, 0.90, 0.82, 0.72))
    dark_tip = Material("dark_temple_tip", rgba=(0.025, 0.023, 0.022, 1.0))

    lens_radius = 0.024
    rim_wire = 0.00115
    center_x = 0.032
    hinge_x = 0.060
    hinge_y = -0.001

    front = model.part("front")

    # Round metal rims with lightly tinted lenses captured just inside the wire.
    for side, cx in (("left", -center_x), ("right", center_x)):
        front.visual(
            _wire_mesh(_circle_points(cx, lens_radius), f"{side}_rim_mesh", radius=rim_wire, closed=True),
            material=polished_steel,
            name=f"{side}_rim",
        )
        front.visual(
            Cylinder(radius=0.0232, length=0.0014),
            origin=Origin(xyz=(cx, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pale_lens,
            name=f"{side}_lens",
        )

    # Arched nose bridge, connected into the inner upper quadrants of the rims.
    bridge_points = [
        (-0.0102, 0.0, 0.0100),
        (-0.0050, -0.0005, 0.0165),
        (0.0050, -0.0005, 0.0165),
        (0.0102, 0.0, 0.0100),
    ]
    front.visual(
        _wire_mesh(bridge_points, "nose_bridge_mesh", radius=0.00105),
        material=polished_steel,
        name="nose_bridge",
    )

    # Two delicate brackets carry soft oval nose pads below the bridge.
    pad_specs = (
        ("left", -0.0075, -0.0062, -0.0115, -0.35),
        ("right", 0.0075, -0.0062, -0.0115, 0.35),
    )
    for side, px, py, pz, yaw in pad_specs:
        sx = -1.0 if side == "left" else 1.0
        bracket_points = [
            (sx * 0.0100, -0.0002, -0.0038),
            (sx * 0.0102, -0.0030, -0.0068),
            (px, py, pz + 0.0020),
        ]
        front.visual(
            _wire_mesh(bracket_points, f"{side}_pad_bracket_mesh", radius=0.00055),
            material=polished_steel,
            name=f"{side}_pad_bracket",
        )
        front.visual(
            _pad_mesh(f"{side}_nose_pad_mesh"),
            origin=Origin(xyz=(px, py, pz), rpy=(math.pi / 2.0, 0.0, yaw)),
            material=soft_silicone,
            name=f"{side}_nose_pad",
        )

    # Compact three-knuckle hinge stacks on each outside edge.  The middle
    # knuckle belongs to the moving temple arm; top and bottom knuckles stay
    # with the front frame.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        front.visual(
            _wire_mesh(
                [
                    (sx * (center_x + lens_radius - 0.0004), 0.0, 0.0),
                    (sx * (hinge_x + 0.0027), hinge_y, 0.0),
                ],
                f"{side}_hinge_lug_mesh",
                radius=0.00125,
            ),
            material=polished_steel,
            name=f"{side}_hinge_lug",
        )
        front.visual(
            Box((0.0014, 0.0040, 0.0220)),
            origin=Origin(xyz=(sx * (hinge_x + 0.0027), hinge_y, 0.0)),
            material=polished_steel,
            name=f"{side}_hinge_backplate",
        )
        for zc, label in ((0.0088, "upper"), (-0.0088, "lower")):
            front.visual(
                Cylinder(radius=0.00215, length=0.0052),
                origin=Origin(xyz=(sx * hinge_x, hinge_y, zc)),
                material=polished_steel,
                name=f"{side}_{label}_hinge_knuckle",
            )

    def make_temple(side: str, sx: float):
        temple = model.part(f"{side}_temple")
        # The temple part frame is the hinge pin.  At q=0 the arm extends
        # backward along -Y; it bows slightly outward and hooks down at the ear.
        arm_points = [
            (0.0, -0.0012, 0.0),
            (sx * 0.0028, -0.020, 0.0002),
            (sx * 0.0052, -0.070, -0.0020),
            (sx * 0.0050, -0.116, -0.0110),
            (sx * 0.0025, -0.140, -0.0250),
        ]
        temple.visual(
            Cylinder(radius=0.00175, length=0.0100),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=polished_steel,
            name="hinge_knuckle",
        )
        temple.visual(
            Box((0.0038, 0.0130, 0.0022)),
            origin=Origin(xyz=(sx * 0.0003, -0.0068, 0.0)),
            material=polished_steel,
            name="hinge_leaf",
        )
        temple.visual(
            _wire_mesh(arm_points, f"{side}_temple_wire_mesh", radius=0.00095),
            material=polished_steel,
            name="temple_wire",
        )
        temple.visual(
            _wire_mesh(arm_points[-3:], f"{side}_ear_tip_mesh", radius=0.00145),
            material=dark_tip,
            name="ear_tip",
        )
        axis = (0.0, 0.0, 1.0) if side == "left" else (0.0, 0.0, -1.0)
        model.articulation(
            f"{side}_hinge",
            ArticulationType.REVOLUTE,
            parent=front,
            child=temple,
            origin=Origin(xyz=(sx * hinge_x, hinge_y, 0.0)),
            axis=axis,
            motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
        )

    make_temple("left", -1.0)
    make_temple("right", 1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front = object_model.get_part("front")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

    hinges = (left_hinge, right_hinge)
    ctx.check(
        "both temple hinges are vertical revolutes",
        all(j.articulation_type == ArticulationType.REVOLUTE and abs(j.axis[2]) > 0.95 for j in hinges),
        details=f"axes={[j.axis for j in hinges]}",
    )
    ctx.check(
        "hinges fold about ninety degrees",
        all(
            j.motion_limits is not None
            and abs(j.motion_limits.lower - 0.0) < 1e-6
            and abs(j.motion_limits.upper - math.pi / 2.0) < 1e-6
            for j in hinges
        ),
        details=f"limits={[j.motion_limits for j in hinges]}",
    )

    # The middle moving knuckle is coaxial with fixed upper/lower hinge knuckles
    # and separated by small real hinge clearances rather than broad overlap.
    for side, temple in (("left", left_temple), ("right", right_temple)):
        ctx.expect_overlap(
            front,
            temple,
            axes="xy",
            min_overlap=0.002,
            elem_a=f"{side}_upper_hinge_knuckle",
            elem_b="hinge_knuckle",
            name=f"{side} hinge knuckles share pin footprint",
        )
        ctx.expect_gap(
            front,
            temple,
            axis="z",
            min_gap=0.0005,
            max_gap=0.003,
            positive_elem=f"{side}_upper_hinge_knuckle",
            negative_elem="hinge_knuckle",
            name=f"{side} upper knuckle clearance",
        )
        ctx.expect_gap(
            temple,
            front,
            axis="z",
            min_gap=0.0005,
            max_gap=0.003,
            positive_elem="hinge_knuckle",
            negative_elem=f"{side}_lower_hinge_knuckle",
            name=f"{side} lower knuckle clearance",
        )

    def elem_center(part, elem):
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        left_open = elem_center(left_temple, "ear_tip")
        right_open = elem_center(right_temple, "ear_tip")
    with ctx.pose({left_hinge: math.pi / 2.0, right_hinge: math.pi / 2.0}):
        left_folded = elem_center(left_temple, "ear_tip")
        right_folded = elem_center(right_temple, "ear_tip")

    ctx.check(
        "temple tips start behind the front frame",
        left_open is not None
        and right_open is not None
        and left_open[1] < -0.10
        and right_open[1] < -0.10,
        details=f"left_open={left_open}, right_open={right_open}",
    )
    ctx.check(
        "left temple folds inward across the lenses",
        left_open is not None
        and left_folded is not None
        and left_folded[0] > left_open[0] + 0.09
        and -0.025 < left_folded[1] < 0.015,
        details=f"left_open={left_open}, left_folded={left_folded}",
    )
    ctx.check(
        "right temple folds inward across the lenses",
        right_open is not None
        and right_folded is not None
        and right_folded[0] < right_open[0] - 0.09
        and -0.025 < right_folded[1] < 0.015,
        details=f"right_open={right_open}, right_folded={right_folded}",
    )

    return ctx.report()


object_model = build_object_model()
