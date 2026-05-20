from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


PIVOT_X = -0.026
PIVOT_Y = 0.020
SWIVEL_OPEN = math.radians(92.0)


def _box(part, name: str, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_swivel_usb_drive")

    rubber = model.material("black_overmolded_rubber", rgba=(0.025, 0.028, 0.026, 1.0))
    ribbed_rubber = model.material("raised_rubber_ribs", rgba=(0.055, 0.060, 0.055, 1.0))
    stainless = model.material("brushed_stainless_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("black_oxide_fasteners", rgba=(0.025, 0.024, 0.022, 1.0))
    cover_yellow = model.material("safety_yellow_powdercoat", rgba=(0.95, 0.69, 0.05, 1.0))
    worn_edges = model.material("worn_reinforced_edges", rgba=(0.82, 0.58, 0.04, 1.0))
    red = model.material("lockout_red", rgba=(0.85, 0.06, 0.035, 1.0))
    gold = model.material("contact_gold", rgba=(1.0, 0.74, 0.18, 1.0))
    black_plastic = model.material("black_connector_tongue", rgba=(0.005, 0.005, 0.006, 1.0))

    body = model.part("drive_body")

    # Armored USB body, with load carried by visible ribs and side guards.
    _box(body, "main_overmold", (0.078, 0.024, 0.012), (-0.008, 0.0, 0.006), rubber)
    _box(body, "rear_bumper", (0.011, 0.028, 0.014), (-0.049, 0.0, 0.007), ribbed_rubber)
    _box(body, "front_bezel", (0.007, 0.028, 0.014), (0.0305, 0.0, 0.007), dark_metal)
    _box(body, "top_spine", (0.060, 0.006, 0.003), (-0.008, 0.0, 0.0135), ribbed_rubber)
    _box(body, "side_guard_0", (0.066, 0.004, 0.006), (-0.008, -0.014, 0.007), ribbed_rubber)
    _box(body, "side_guard_1", (0.066, 0.004, 0.006), (-0.008, 0.014, 0.007), ribbed_rubber)
    _box(body, "front_guard_0", (0.024, 0.003, 0.007), (0.041, -0.0135, 0.0065), dark_metal)
    _box(body, "front_guard_1", (0.024, 0.003, 0.007), (0.041, 0.0135, 0.0065), dark_metal)

    # Open USB-A plug: side walls, top/bottom shell, rear web, tongue and contacts.
    _box(body, "connector_top", (0.020, 0.013, 0.0012), (0.041, 0.0, 0.0096), stainless)
    _box(body, "connector_bottom", (0.020, 0.013, 0.0012), (0.041, 0.0, 0.0024), stainless)
    _box(body, "connector_side_0", (0.020, 0.0012, 0.0062), (0.041, -0.0066, 0.006), stainless)
    _box(body, "connector_side_1", (0.020, 0.0012, 0.0062), (0.041, 0.0066, 0.006), stainless)
    _box(body, "connector_rear_web", (0.0026, 0.013, 0.0082), (0.0317, 0.0, 0.006), stainless)
    _box(body, "tongue_root", (0.005, 0.006, 0.0030), (0.0345, 0.0, 0.006), black_plastic)
    _box(body, "connector_tongue", (0.016, 0.006, 0.0016), (0.0438, 0.0, 0.006), black_plastic)
    for idx, y in enumerate((-0.00225, -0.00075, 0.00075, 0.00225)):
        _box(body, f"gold_contact_{idx}", (0.0048, 0.0008, 0.00025), (0.043, y, 0.006925), gold)

    # Side-pinned pivot mount. The yellow cover eye rotates around the visible pin,
    # while body-side washers and bearing blocks show the load path into the housing.
    _box(body, "pivot_side_boss", (0.026, 0.011, 0.010), (PIVOT_X, 0.0170, 0.006), dark_metal)
    _box(body, "pivot_lower_plate", (0.030, 0.014, 0.003), (PIVOT_X, 0.0180, 0.0105), dark_metal)
    _cyl(body, "pivot_pin", 0.0024, 0.024, (PIVOT_X, PIVOT_Y, 0.012), stainless)
    _cyl(body, "lower_washer", 0.0058, 0.0012, (PIVOT_X, PIVOT_Y, 0.0116), dark_metal)
    _cyl(body, "upper_washer", 0.0058, 0.0012, (PIVOT_X, PIVOT_Y, 0.0172), dark_metal)
    _cyl(body, "pin_peened_head", 0.0036, 0.0012, (PIVOT_X, PIVOT_Y, 0.0246), dark_metal)
    _box(body, "pin_guard_upright_0", (0.004, 0.004, 0.014), (PIVOT_X - 0.011, PIVOT_Y + 0.010, 0.017), dark_metal)
    _box(body, "pin_guard_upright_1", (0.004, 0.004, 0.014), (PIVOT_X + 0.011, PIVOT_Y + 0.010, 0.017), dark_metal)
    _box(body, "pin_guard_foot_0", (0.004, 0.006, 0.003), (PIVOT_X - 0.011, PIVOT_Y + 0.0052, 0.0105), dark_metal)
    _box(body, "pin_guard_foot_1", (0.004, 0.006, 0.003), (PIVOT_X + 0.011, PIVOT_Y + 0.0052, 0.0105), dark_metal)
    _box(body, "pin_guard_bridge", (0.026, 0.004, 0.003), (PIVOT_X, PIVOT_Y + 0.010, 0.0255), dark_metal)

    # Positive mechanical stop anvils, not just software limits.
    _box(body, "closed_stop_anvil", (0.006, 0.011, 0.007), (PIVOT_X + 0.018, PIVOT_Y + 0.006, 0.0155), dark_metal)
    _box(body, "open_stop_arm", (0.006, 0.020, 0.004), (PIVOT_X - 0.006, PIVOT_Y + 0.012, 0.0125), dark_metal)
    _box(body, "open_stop_anvil", (0.011, 0.006, 0.007), (PIVOT_X - 0.006, PIVOT_Y + 0.024, 0.0155), dark_metal)

    # Captive rail for a red lockout slider.
    _box(body, "lockout_rail_0", (0.026, 0.002, 0.003), (-0.034, -0.0115, 0.0140), dark_metal)
    _box(body, "lockout_rail_1", (0.026, 0.002, 0.003), (-0.034, 0.0005, 0.0140), dark_metal)
    _box(body, "lockout_rear_stop", (0.0025, 0.014, 0.004), (-0.048, -0.0055, 0.0140), dark_metal)
    _box(body, "lockout_front_stop", (0.0025, 0.014, 0.004), (-0.020, -0.0055, 0.0140), dark_metal)

    for idx, (x, y) in enumerate(((-0.038, -0.008), (-0.038, 0.008), (0.018, -0.008), (0.018, 0.008))):
        _cyl(body, f"body_screw_{idx}", 0.0021, 0.0010, (x, y, 0.0125), dark_metal)
        _box(body, f"body_screw_slot_{idx}", (0.0032, 0.00055, 0.0003), (x, y, 0.01315), rubber)

    cover = model.part("swivel_cover")
    # The child frame is the pin axis. At q=0 the cover protects the connector;
    # positive yaw swings the cover out to the side, exposing the plug.
    _box(cover, "top_plate", (0.072, 0.033, 0.003), (0.044, -0.020, 0.0175), cover_yellow)
    _box(cover, "side_skirt_0", (0.065, 0.003, 0.010), (0.0475, -0.038, 0.0120), cover_yellow)
    _box(cover, "side_skirt_1", (0.065, 0.003, 0.010), (0.0475, -0.002, 0.0120), cover_yellow)
    _box(cover, "nose_crash_bar", (0.005, 0.033, 0.010), (0.082, -0.020, 0.0125), worn_edges)
    _box(cover, "front_lip", (0.010, 0.033, 0.003), (0.076, -0.020, 0.0188), worn_edges)
    _box(cover, "center_spine", (0.062, 0.005, 0.003), (0.045, -0.020, 0.0210), worn_edges)
    _box(cover, "edge_stiffener_0", (0.062, 0.004, 0.0025), (0.046, -0.034, 0.0203), worn_edges)
    _box(cover, "edge_stiffener_1", (0.062, 0.004, 0.0025), (0.046, -0.006, 0.0203), worn_edges)
    _box(cover, "diagonal_brace_0", (0.058, 0.003, 0.0024), (0.045, -0.020, 0.0230), dark_metal, rpy=(0.0, 0.0, 0.34))
    _box(cover, "diagonal_brace_1", (0.058, 0.003, 0.0024), (0.045, -0.020, 0.0243), dark_metal, rpy=(0.0, 0.0, -0.34))

    # Four bars form a real pivot eye around, but clear of, the pin.
    _box(cover, "eye_front", (0.004, 0.019, 0.004), (0.007, 0.0, 0.0140), cover_yellow)
    _box(cover, "eye_rear", (0.004, 0.019, 0.004), (-0.007, 0.0, 0.0140), cover_yellow)
    _box(cover, "eye_inboard", (0.019, 0.004, 0.004), (0.0, -0.007, 0.0140), cover_yellow)
    _box(cover, "eye_outboard", (0.019, 0.004, 0.004), (0.0, 0.007, 0.0140), cover_yellow)
    _box(cover, "closed_stop_tab", (0.004, 0.012, 0.005), (0.014, 0.005, 0.0145), dark_metal)
    _box(cover, "open_stop_web", (0.004, 0.005, 0.005), (-0.004, 0.0105, 0.0145), dark_metal)
    _box(cover, "open_stop_tab", (0.012, 0.004, 0.005), (-0.004, 0.014, 0.0145), dark_metal)
    _box(cover, "lockout_notch_plate", (0.014, 0.012, 0.0015), (0.020, -0.020, 0.0202), dark_metal)
    for idx, (x, y) in enumerate(((0.020, -0.032), (0.020, -0.008), (0.066, -0.032), (0.066, -0.008))):
        _cyl(cover, f"cover_screw_{idx}", 0.0018, 0.0010, (x, y, 0.0219), dark_metal)
        _box(cover, f"cover_screw_slot_{idx}", (0.0028, 0.0005, 0.00028), (x, y, 0.02254), cover_yellow)

    lockout = model.part("lockout_slider")
    _box(lockout, "slider_plate", (0.014, 0.0085, 0.0030), (0.0, 0.0, 0.0015), red)
    _box(lockout, "thumb_rib", (0.009, 0.0045, 0.0022), (0.001, 0.0, 0.0041), red)
    _box(lockout, "latch_tooth", (0.004, 0.004, 0.0030), (0.008, 0.0, 0.0020), dark_metal)

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=SWIVEL_OPEN),
        motion_properties=MotionProperties(damping=0.08, friction=0.18),
    )

    model.articulation(
        "body_to_lockout",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lockout,
        origin=Origin(xyz=(-0.034, -0.0055, 0.0120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.006),
        motion_properties=MotionProperties(damping=0.12, friction=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("drive_body")
    cover = object_model.get_part("swivel_cover")
    lockout = object_model.get_part("lockout_slider")
    cover_joint = object_model.get_articulation("body_to_cover")
    lockout_joint = object_model.get_articulation("body_to_lockout")

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="top_plate",
            elem_b="connector_top",
            min_overlap=0.010,
            name="closed cover spans connector footprint",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_plate",
            negative_elem="connector_top",
            min_gap=0.004,
            max_gap=0.008,
            name="closed cover clears connector above",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="z",
            elem_a="eye_front",
            elem_b="pivot_pin",
            min_overlap=0.003,
            name="pivot eye is vertically captured by visible pin",
        )

    with ctx.pose({cover_joint: SWIVEL_OPEN}):
        ctx.expect_gap(
            cover,
            body,
            axis="y",
            positive_elem="top_plate",
            negative_elem="connector_top",
            min_gap=0.010,
            name="open cover swings clear of connector",
        )

    rest_pos = ctx.part_world_position(lockout)
    with ctx.pose({lockout_joint: 0.006}):
        extended_pos = ctx.part_world_position(lockout)
    ctx.check(
        "lockout slider travels in guarded rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.005,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
