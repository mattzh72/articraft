from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


REMOTE_LENGTH = 0.190
REMOTE_WIDE = 0.054
REMOTE_NARROW = 0.043
REMOTE_THICKNESS = 0.018
BODY_TOP_Z = REMOTE_THICKNESS / 2.0
BODY_BOTTOM_Z = -REMOTE_THICKNESS / 2.0

PANEL_TOP_Z = BODY_TOP_Z + 0.00085


def _tapered_body_shape() -> cq.Workplane:
    """A gently tapered hand-held remote body with softened plastic edges."""

    half_len = REMOTE_LENGTH / 2.0
    outline = [
        (-REMOTE_WIDE / 2.0, -half_len),
        (REMOTE_WIDE / 2.0, -half_len),
        (REMOTE_NARROW / 2.0, half_len),
        (-REMOTE_NARROW / 2.0, half_len),
    ]
    # CadQuery's ``both=True`` extrudes the requested distance in both directions,
    # so use half the desired product thickness to land at +/- REMOTE_THICKNESS/2.
    body = cq.Workplane("XY").polyline(outline).close().extrude(REMOTE_THICKNESS / 2.0, both=True)

    # Product-like molded plastic: radiused plan corners and lightly broken top/bottom lips.
    try:
        body = body.edges("|Z").fillet(0.006)
        body = body.edges(">Z or <Z").fillet(0.0018)
    except Exception:
        # Keep the taper if a fillet selector becomes over-constrained on a backend version.
        pass
    return body


def _rounded_box_mesh(width: float, length: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, length, radius, corner_segments=8),
            height,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tapered_tv_remote")

    dark_plastic = model.material("satin_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    panel_black = model.material("glossy_black_panel", rgba=(0.0, 0.0, 0.0, 1.0))
    rubber = model.material("dark_rubber_buttons", rgba=(0.045, 0.047, 0.050, 1.0))
    label_gray = model.material("soft_gray_buttons", rgba=(0.18, 0.19, 0.20, 1.0))
    red = model.material("red_power_button", rgba=(0.75, 0.03, 0.025, 1.0))
    door_mat = model.material("slightly_lighter_battery_door", rgba=(0.028, 0.030, 0.034, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tapered_body_shape(), "tapered_remote_body", tolerance=0.0006),
        material=dark_plastic,
        name="tapered_shell",
    )
    body.visual(
        _rounded_box_mesh(0.040, 0.148, 0.0010, 0.006, "top_control_panel"),
        origin=Origin(xyz=(0.0, 0.004, BODY_TOP_Z + 0.00035)),
        material=panel_black,
        name="top_panel",
    )

    # Underside slide rails and an end stop make the battery cover read as retained.
    rail_y = -0.040
    for idx, x in enumerate((-0.0205, 0.0205)):
        body.visual(
            Box((0.0040, 0.074, 0.0022)),
            origin=Origin(xyz=(x, rail_y, BODY_BOTTOM_Z - 0.0010)),
            material=dark_plastic,
            name=f"battery_rail_{idx}",
        )
    body.visual(
        Box((0.034, 0.0030, 0.0020)),
        origin=Origin(xyz=(0.0, -0.0005, BODY_BOTTOM_Z - 0.0010)),
        material=dark_plastic,
        name="battery_stop_lip",
    )

    def add_round_button(name: str, x: float, y: float, radius: float, height: float, material) -> None:
        button = model.part(name)
        button.visual(
            Cylinder(radius=radius, length=height),
            origin=Origin(),
            material=material,
            name="cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, y, PANEL_TOP_Z + height / 2.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.05, lower=0.0, upper=0.0022),
        )

    def add_rounded_button(
        name: str,
        x: float,
        y: float,
        width: float,
        length: float,
        height: float,
        material,
    ) -> None:
        button = model.part(name)
        button.visual(
            _rounded_box_mesh(width, length, height, min(width, length) * 0.35, f"{name}_cap"),
            origin=Origin(),
            material=material,
            name="cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, y, PANEL_TOP_Z + height / 2.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.05, lower=0.0, upper=0.0024),
        )

    add_round_button("power_button", -0.0135, 0.070, 0.0048, 0.0036, red)
    add_round_button("input_button", 0.0135, 0.070, 0.0045, 0.0034, label_gray)
    add_rounded_button("volume_button", -0.0125, 0.043, 0.0100, 0.027, 0.0038, label_gray)
    add_rounded_button("channel_button", 0.0125, 0.043, 0.0100, 0.027, 0.0038, label_gray)

    x_positions = (-0.012, 0.0, 0.012)
    y_positions = (0.012, -0.006, -0.024, -0.042)
    for row, y in enumerate(y_positions):
        for col, x in enumerate(x_positions):
            add_round_button(f"button_{row}_{col}", x, y, 0.0042, 0.0034, rubber)

    battery_door = model.part("battery_door")
    battery_door.visual(
        _rounded_box_mesh(0.031, 0.061, 0.0020, 0.0045, "battery_door_plate"),
        origin=Origin(),
        material=door_mat,
        name="door_plate",
    )
    for idx, x in enumerate((-0.0170, 0.0170)):
        battery_door.visual(
            Box((0.0030, 0.059, 0.0016)),
            origin=Origin(xyz=(x, 0.0, -0.00015)),
            material=door_mat,
            name=f"side_tongue_{idx}",
        )
    for idx, y in enumerate((-0.012, 0.0, 0.012)):
        battery_door.visual(
            Box((0.019, 0.0022, 0.0008)),
            origin=Origin(xyz=(0.0, y, -0.00135)),
            material=dark_plastic,
            name=f"grip_ridge_{idx}",
        )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, -0.040, BODY_BOTTOM_Z - 0.0020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.12, lower=0.0, upper=0.025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    power = object_model.get_part("power_button")
    number = object_model.get_part("button_2_1")
    other_number = object_model.get_part("button_2_2")
    door = object_model.get_part("battery_door")
    power_slide = object_model.get_articulation("body_to_power_button")
    number_slide = object_model.get_articulation("body_to_button_2_1")
    door_slide = object_model.get_articulation("body_to_battery_door")

    ctx.expect_gap(
        power,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="raised power button is seated on the top panel",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xy",
        min_overlap=0.025,
        name="battery door lies on the remote back footprint",
    )

    power_rest = ctx.part_world_position(power)
    with ctx.pose({power_slide: 0.0022}):
        power_down = ctx.part_world_position(power)
    ctx.check(
        "power button depresses inward",
        power_rest is not None
        and power_down is not None
        and power_down[2] < power_rest[2] - 0.0018,
        details=f"rest={power_rest}, depressed={power_down}",
    )

    number_rest = ctx.part_world_position(number)
    neighbor_rest = ctx.part_world_position(other_number)
    with ctx.pose({number_slide: 0.0022}):
        number_down = ctx.part_world_position(number)
        neighbor_still = ctx.part_world_position(other_number)
    ctx.check(
        "one number button depresses independently",
        number_rest is not None
        and number_down is not None
        and neighbor_rest is not None
        and neighbor_still is not None
        and number_down[2] < number_rest[2] - 0.0018
        and abs(neighbor_still[2] - neighbor_rest[2]) < 0.0002,
        details=(
            f"number_rest={number_rest}, number_down={number_down}, "
            f"neighbor_rest={neighbor_rest}, neighbor_still={neighbor_still}"
        ),
    )

    door_rest = ctx.part_world_position(door)
    with ctx.pose({door_slide: 0.025}):
        door_open = ctx.part_world_position(door)
        ctx.expect_overlap(
            door,
            body,
            axes="y",
            min_overlap=0.035,
            name="sliding battery door remains retained in the rails",
        )
    ctx.check(
        "battery door slides along the back",
        door_rest is not None and door_open is not None and door_open[1] < door_rest[1] - 0.020,
        details=f"rest={door_rest}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()
