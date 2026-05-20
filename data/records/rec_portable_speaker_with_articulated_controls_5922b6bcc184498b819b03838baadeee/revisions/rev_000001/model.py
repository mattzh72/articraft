from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


BODY_RADIUS = 0.075
BODY_LENGTH = 0.340
PIVOT_Z = 0.106
SPINE_TOP_Z = 0.087
BUTTON_Z = SPINE_TOP_Z + 0.0005
BUTTON_TRAVEL = 0.0025


def _handle_frame_geometry() -> MeshGeometry:
    """One connected molded-plastic bail handle frame, local to the hinge axis."""
    geom = MeshGeometry()

    # Horizontal tube across the top of the handle, aligned with the hinge axis.
    geom.merge(
        CylinderGeometry(0.008, 0.300, radial_segments=32)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, 0.066)
    )

    # Two vertical arms down to the trunnion pins.
    for x in (-0.150, 0.150):
        geom.merge(CylinderGeometry(0.007, 0.064, radial_segments=28).translate(x, 0.0, 0.036))
        geom.merge(SphereGeometry(0.010, width_segments=28, height_segments=14).translate(x, 0.0, 0.066))
        geom.merge(SphereGeometry(0.008, width_segments=24, height_segments=12).translate(x, 0.0, 0.005))

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_bluetooth_speaker")

    fabric = model.material("charcoal_fabric", rgba=(0.035, 0.038, 0.040, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    plastic = model.material("dark_graphite_plastic", rgba=(0.10, 0.105, 0.11, 1.0))
    gray = model.material("soft_gray_trim", rgba=(0.32, 0.34, 0.35, 1.0))
    white = model.material("button_icon_white", rgba=(0.88, 0.90, 0.88, 1.0))
    blue = model.material("bluetooth_blue", rgba=(0.05, 0.30, 0.95, 1.0))

    ring_mesh = mesh_from_geometry(TorusGeometry(0.055, 0.0022, radial_segments=48, tubular_segments=10), "end_cap_ring")
    guide_ring_mesh = mesh_from_geometry(TorusGeometry(0.012, 0.0014, radial_segments=32, tubular_segments=8), "button_guide_ring")
    handle_frame_mesh = mesh_from_geometry(_handle_frame_geometry(), "handle_frame")

    body = model.part("body")
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=BODY_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fabric,
        name="main_shell",
    )
    for suffix, x in (("0", -0.174), ("1", 0.174)):
        body.visual(
            Cylinder(radius=0.076, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"end_cap_{suffix}",
        )
        body.visual(
            ring_mesh,
            origin=Origin(xyz=(x + (-0.006 if x < 0 else 0.006), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gray,
            name=f"end_ring_{suffix}",
        )

    body.visual(
        Box((0.230, 0.048, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, SPINE_TOP_Z - 0.009)),
        material=plastic,
        name="top_spine",
    )
    body.visual(
        Box((0.055, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -0.075, 0.0)),
        material=gray,
        name="front_badge",
    )

    # Two fixed hinge brackets at the speaker ends.  Each has a molded base and
    # two bearing collars, leaving a central gap for the moving handle lug.
    pivot_xs = (-0.150, 0.150)
    for bracket_i, pivot_x in enumerate(pivot_xs):
        body.visual(
            Box((0.058, 0.040, 0.030)),
            origin=Origin(xyz=(pivot_x, 0.0, 0.090)),
            material=plastic,
            name=f"bracket_base_{bracket_i}",
        )
        for collar_i, dx in enumerate((-0.020, 0.020)):
            body.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(pivot_x + dx, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=plastic,
                name=f"bracket_{bracket_i}_collar_{collar_i}",
            )

    button_xs = (-0.060, -0.030, 0.0, 0.030, 0.060)
    for i, x in enumerate(button_xs):
        body.visual(
            guide_ring_mesh,
            origin=Origin(xyz=(x, 0.0, SPINE_TOP_Z + 0.0006)),
            material=rubber,
            name=f"button_guide_{i}",
        )

    handle = model.part("handle")
    handle.visual(handle_frame_mesh, material=plastic, name="handle_frame")
    handle.visual(
        Cylinder(radius=0.011, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_sleeve",
    )
    for suffix, x in (("0", -0.150), ("1", 0.150)):
        handle.visual(
            Cylinder(radius=0.0055, length=0.056),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gray,
            name=f"hinge_pin_{suffix}",
        )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.15, upper=1.15),
    )

    def add_button(part_name: str, x: float, icon: str, mat: Material = rubber) -> None:
        button = model.part(part_name)
        button.visual(
            Cylinder(radius=0.009, length=0.0045),
            origin=Origin(xyz=(0.0, 0.0, 0.00425)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.004, length=0.0075),
            origin=Origin(xyz=(0.0, 0.0, -0.00175)),
            material=rubber,
            name="guide_stem",
        )
        icon_z = 0.00675
        if icon == "minus":
            button.visual(Box((0.010, 0.0016, 0.0006)), origin=Origin(xyz=(0.0, 0.0, icon_z)), material=white, name="icon_bar")
        elif icon == "plus":
            button.visual(Box((0.010, 0.0016, 0.0006)), origin=Origin(xyz=(0.0, 0.0, icon_z)), material=white, name="icon_bar")
            button.visual(Box((0.0016, 0.010, 0.0006)), origin=Origin(xyz=(0.0, 0.0, icon_z)), material=white, name="icon_cross")
        elif icon == "play":
            button.visual(Box((0.003, 0.010, 0.0006)), origin=Origin(xyz=(-0.0025, 0.0, icon_z)), material=white, name="icon_pause_0")
            button.visual(Box((0.003, 0.010, 0.0006)), origin=Origin(xyz=(0.0025, 0.0, icon_z)), material=white, name="icon_pause_1")
        elif icon == "power":
            button.visual(Box((0.0018, 0.010, 0.0006)), origin=Origin(xyz=(0.0, 0.0, icon_z)), material=white, name="icon_line")
        elif icon == "bt":
            button.visual(Box((0.0020, 0.011, 0.0006)), origin=Origin(xyz=(0.0, 0.0, icon_z)), material=white, name="icon_stem")
            button.visual(Box((0.008, 0.0015, 0.0006)), origin=Origin(xyz=(0.0, 0.0035, icon_z), rpy=(0.0, 0.0, 0.65)), material=white, name="icon_slash_0")
            button.visual(Box((0.008, 0.0015, 0.0006)), origin=Origin(xyz=(0.0, -0.0035, icon_z), rpy=(0.0, 0.0, -0.65)), material=white, name="icon_slash_1")

        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, 0.0, BUTTON_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
        )

    add_button("power_button", button_xs[0], "power")
    add_button("minus_button", button_xs[1], "minus")
    add_button("play_button", button_xs[2], "play")
    add_button("plus_button", button_xs[3], "plus")
    add_button("bt_button", button_xs[4], "bt", mat=blue)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("body_to_handle")

    # The hinge pins are intentionally captured inside the fixed bearing collars.
    for bracket_i, pin_i in ((0, 0), (1, 1)):
        for collar_i in (0, 1):
            collar = f"bracket_{bracket_i}_collar_{collar_i}"
            pin = f"hinge_pin_{pin_i}"
            ctx.allow_overlap(
                body,
                handle,
                elem_a=collar,
                elem_b=pin,
                reason="The handle hinge pin is intentionally captured by the fixed bracket bearing collar.",
            )
            ctx.expect_within(
                handle,
                body,
                axes="yz",
                inner_elem=pin,
                outer_elem=collar,
                margin=0.0,
                name=f"{pin} centered in {collar}",
            )
            ctx.expect_overlap(
                handle,
                body,
                axes="x",
                elem_a=pin,
                elem_b=collar,
                min_overlap=0.006,
                name=f"{pin} retained by {collar}",
            )

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        min_gap=0.015,
        positive_elem="handle_frame",
        negative_elem="main_shell",
        name="upright handle clears cylindrical body",
    )

    rest_aabb = ctx.part_element_world_aabb(handle, elem="handle_frame")
    with ctx.pose({handle_joint: 0.9}):
        swung_aabb = ctx.part_element_world_aabb(handle, elem="handle_frame")
    if rest_aabb is not None and swung_aabb is not None:
        rest_y = (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
        swung_y = (swung_aabb[0][1] + swung_aabb[1][1]) * 0.5
    else:
        rest_y = swung_y = None
    ctx.check(
        "handle rotates about side hinge axis",
        rest_y is not None and swung_y is not None and swung_y < rest_y - 0.025,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    button_names = ("power_button", "minus_button", "play_button", "plus_button", "bt_button")
    for button_name in button_names:
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(f"body_to_{button_name}")
        ctx.allow_overlap(
            body,
            button,
            elem_a="top_spine",
            elem_b="guide_stem",
            reason="The pushbutton guide stem is intentionally represented as sliding inside the upper housing spine.",
        )
        ctx.expect_within(
            button,
            body,
            axes="xy",
            inner_elem="guide_stem",
            outer_elem="top_spine",
            margin=0.0,
            name=f"{button_name} guide stem centered in spine",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="z",
            elem_a="guide_stem",
            elem_b="top_spine",
            min_overlap=0.003,
            name=f"{button_name} guide stem retained",
        )
        ctx.expect_gap(
            button,
            body,
            axis="z",
            min_gap=0.0015,
            max_gap=0.0035,
            positive_elem="button_cap",
            negative_elem="top_spine",
            name=f"{button_name} stands proud of upper spine",
        )
        ctx.expect_within(
            button,
            body,
            axes="xy",
            inner_elem="button_cap",
            outer_elem="top_spine",
            margin=0.0,
            name=f"{button_name} within upper spine",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_gap(
                button,
                body,
                axis="z",
                max_gap=0.0006,
                max_penetration=0.0002,
                positive_elem="button_cap",
                negative_elem="top_spine",
                name=f"{button_name} presses nearly flush",
            )
        ctx.check(
            f"{button_name} depresses into housing",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
