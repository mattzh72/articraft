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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bank_end_slot_machine")

    cabinet_black = model.material("cabinet_black", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_graphite = model.material("trim_graphite", rgba=(0.20, 0.21, 0.24, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.64, 0.66, 0.70, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.08, 0.13, 0.16, 0.68))
    display_black = model.material("display_black", rgba=(0.03, 0.04, 0.05, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.14, 0.52, 0.80, 1.0))

    cabinet_body = model.part("cabinet_body")
    cabinet_body.inertial = Inertial.from_geometry(
        Box((0.82, 0.72, 1.94)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
    )

    cabinet_body.visual(
        Box((0.82, 0.72, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=trim_graphite,
        name="base_plinth",
    )
    cabinet_body.visual(
        Box((0.72, 0.58, 0.76)),
        origin=Origin(xyz=(0.0, -0.05, 0.46)),
        material=cabinet_black,
        name="lower_cabinet_core",
    )
    cabinet_body.visual(
        Box((0.66, 0.28, 0.12)),
        origin=Origin(xyz=(0.0, 0.10, 0.82)),
        material=trim_graphite,
        name="console_bridge",
    )
    cabinet_body.visual(
        Box((0.58, 0.018, 0.16)),
        origin=Origin(xyz=(0.0, 0.200, 0.828), rpy=(-0.52, 0.0, 0.0)),
        material=satin_silver,
        name="sloped_console_face",
    )
    cabinet_body.visual(
        Box((0.62, 0.28, 0.48)),
        origin=Origin(xyz=(0.0, 0.10, 1.08)),
        material=cabinet_black,
        name="lower_screen_housing",
    )
    cabinet_body.visual(
        Box((0.58, 0.24, 0.44)),
        origin=Origin(xyz=(0.0, 0.08, 1.54)),
        material=trim_graphite,
        name="upper_screen_housing",
    )
    cabinet_body.visual(
        Box((0.50, 0.20, 0.18)),
        origin=Origin(xyz=(0.0, 0.05, 1.85)),
        material=cabinet_black,
        name="marquee_housing",
    )
    cabinet_body.visual(
        Box((0.024, 0.22, 0.64)),
        origin=Origin(xyz=(0.31, 0.145, 1.19), rpy=(-0.24, 0.0, 0.0)),
        material=trim_graphite,
        name="right_stack_cheek",
    )
    cabinet_body.visual(
        Box((0.024, 0.22, 0.64)),
        origin=Origin(xyz=(-0.31, 0.145, 1.19), rpy=(-0.24, 0.0, 0.0)),
        material=trim_graphite,
        name="left_stack_cheek",
    )
    cabinet_body.visual(
        Box((0.44, 0.035, 0.05)),
        origin=Origin(xyz=(0.0, 0.12, 1.938), rpy=(-0.20, 0.0, 0.0)),
        material=accent_blue,
        name="marquee_crown_light",
    )
    cabinet_body.visual(
        Box((0.56, 0.018, 0.42)),
        origin=Origin(xyz=(0.0, 0.249, 1.08)),
        material=satin_silver,
        name="lower_screen_bezel",
    )
    cabinet_body.visual(
        Box((0.52, 0.006, 0.38)),
        origin=Origin(xyz=(0.0, 0.243, 1.08)),
        material=smoked_glass,
        name="lower_screen_glass",
    )
    cabinet_body.visual(
        Box((0.52, 0.010, 0.04)),
        origin=Origin(xyz=(0.0, 0.245, 0.855)),
        material=accent_blue,
        name="player_light_bar",
    )
    cabinet_body.visual(
        Box((0.52, 0.016, 0.34)),
        origin=Origin(xyz=(0.0, 0.209, 1.54)),
        material=satin_silver,
        name="upper_screen_bezel",
    )
    cabinet_body.visual(
        Box((0.46, 0.006, 0.30)),
        origin=Origin(xyz=(0.0, 0.203, 1.54)),
        material=smoked_glass,
        name="upper_screen_glass",
    )
    cabinet_body.visual(
        Box((0.42, 0.016, 0.11)),
        origin=Origin(xyz=(0.0, 0.157, 1.85)),
        material=satin_silver,
        name="marquee_bezel",
    )
    cabinet_body.visual(
        Box((0.38, 0.006, 0.08)),
        origin=Origin(xyz=(0.0, 0.153, 1.85)),
        material=display_black,
        name="marquee_panel",
    )

    acceptor_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.30, 0.24, 0.018, corner_segments=8),
        [
            _offset_profile(
                rounded_rect_profile(0.13, 0.05, 0.007, corner_segments=6),
                dy=0.05,
            ),
            _offset_profile(
                rounded_rect_profile(0.16, 0.038, 0.006, corner_segments=6),
                dy=-0.05,
            ),
        ],
        height=0.02,
        center=True,
    ).rotate_x(pi / 2.0)
    cabinet_body.visual(
        _mesh("slot_machine_acceptor_panel", acceptor_panel_geom),
        origin=Origin(xyz=(0.0, 0.25, 0.88)),
        material=satin_silver,
        name="acceptor_panel",
    )
    cabinet_body.visual(
        Box((0.115, 0.10, 0.065)),
        origin=Origin(xyz=(0.0, 0.19, 0.93)),
        material=display_black,
        name="bill_tunnel",
    )
    cabinet_body.visual(
        Box((0.145, 0.10, 0.055)),
        origin=Origin(xyz=(0.0, 0.19, 0.83)),
        material=display_black,
        name="ticket_tunnel",
    )
    cabinet_body.visual(
        Box((0.20, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.245, 0.745)),
        material=satin_silver,
        name="ticket_tray_trim",
    )
    cabinet_body.visual(
        Box((0.18, 0.012, 0.08)),
        origin=Origin(xyz=(0.0, 0.245, 0.92)),
        material=trim_graphite,
        name="acceptor_surround",
    )

    bill_flap = model.part("bill_flap")
    bill_flap.inertial = Inertial.from_geometry(
        Box((0.125, 0.012, 0.052)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.006, -0.026)),
    )
    bill_flap.visual(
        Box((0.125, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, 0.003, -0.026)),
        material=trim_graphite,
        name="bill_leaf",
    )
    bill_flap.visual(
        Cylinder(radius=0.004, length=0.118),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_silver,
        name="bill_hinge_barrel",
    )
    bill_flap.visual(
        Box((0.105, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.007, -0.048)),
        material=satin_silver,
        name="bill_lip",
    )

    ticket_flap = model.part("ticket_flap")
    ticket_flap.inertial = Inertial.from_geometry(
        Box((0.150, 0.012, 0.040)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.006, -0.020)),
    )
    ticket_flap.visual(
        Box((0.150, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.003, -0.020)),
        material=trim_graphite,
        name="ticket_leaf",
    )
    ticket_flap.visual(
        Cylinder(radius=0.004, length=0.140),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_silver,
        name="ticket_hinge_barrel",
    )
    ticket_flap.visual(
        Box((0.120, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.007, -0.036)),
        material=satin_silver,
        name="ticket_lip",
    )

    service_door = model.part("service_door")
    service_door.inertial = Inertial.from_geometry(
        Box((0.024, 0.48, 0.50)),
        mass=4.5,
        origin=Origin(xyz=(0.012, -0.24, 0.0)),
    )
    service_door.visual(
        Box((0.018, 0.48, 0.50)),
        origin=Origin(xyz=(0.009, -0.24, 0.0)),
        material=trim_graphite,
        name="door_panel",
    )
    service_door.visual(
        Box((0.010, 0.14, 0.030)),
        origin=Origin(xyz=(0.014, -0.12, 0.02)),
        material=satin_silver,
        name="door_pull",
    )
    service_door.visual(
        Box((0.004, 0.18, 0.012)),
        origin=Origin(xyz=(0.011, -0.29, 0.11)),
        material=satin_silver,
        name="door_vent_upper",
    )
    service_door.visual(
        Box((0.004, 0.18, 0.012)),
        origin=Origin(xyz=(0.011, -0.29, 0.075)),
        material=satin_silver,
        name="door_vent_lower",
    )

    model.articulation(
        "bill_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=bill_flap,
        origin=Origin(xyz=(0.0, 0.26, 0.955)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "ticket_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=ticket_flap,
        origin=Origin(xyz=(0.0, 0.26, 0.85)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "service_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=service_door,
        origin=Origin(xyz=(0.36, 0.18, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.35),
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
    cabinet_body = object_model.get_part("cabinet_body")
    bill_flap = object_model.get_part("bill_flap")
    ticket_flap = object_model.get_part("ticket_flap")
    service_door = object_model.get_part("service_door")
    bill_hinge = object_model.get_articulation("bill_flap_hinge")
    ticket_hinge = object_model.get_articulation("ticket_flap_hinge")
    service_hinge = object_model.get_articulation("service_door_hinge")

    ctx.expect_gap(
        bill_flap,
        cabinet_body,
        axis="y",
        positive_elem="bill_leaf",
        negative_elem="acceptor_panel",
        max_gap=0.002,
        max_penetration=0.0,
        name="bill flap sits closed at the upper acceptor opening",
    )
    ctx.expect_gap(
        ticket_flap,
        cabinet_body,
        axis="y",
        positive_elem="ticket_leaf",
        negative_elem="acceptor_panel",
        max_gap=0.002,
        max_penetration=0.0,
        name="ticket flap sits closed at the lower payout opening",
    )
    ctx.expect_gap(
        service_door,
        cabinet_body,
        axis="x",
        positive_elem="door_panel",
        negative_elem="lower_cabinet_core",
        max_gap=0.002,
        max_penetration=0.0,
        name="service door closes flush with the cabinet side",
    )
    ctx.expect_gap(
        bill_flap,
        ticket_flap,
        axis="z",
        positive_elem="bill_leaf",
        negative_elem="ticket_leaf",
        min_gap=0.04,
        name="bill opening remains above the ticket opening",
    )

    rest_bill = ctx.part_element_world_aabb(bill_flap, elem="bill_leaf")
    with ctx.pose({bill_hinge: 1.0}):
        open_bill = ctx.part_element_world_aabb(bill_flap, elem="bill_leaf")
    ctx.check(
        "bill flap rotates outward on a horizontal hinge",
        rest_bill is not None
        and open_bill is not None
        and open_bill[1][1] > rest_bill[1][1] + 0.03
        and open_bill[0][2] > rest_bill[0][2] + 0.015,
        details=f"rest={rest_bill}, open={open_bill}",
    )

    rest_ticket = ctx.part_element_world_aabb(ticket_flap, elem="ticket_leaf")
    with ctx.pose({ticket_hinge: 0.95}):
        open_ticket = ctx.part_element_world_aabb(ticket_flap, elem="ticket_leaf")
    ctx.check(
        "ticket flap rotates outward on its own lower horizontal hinge",
        rest_ticket is not None
        and open_ticket is not None
        and open_ticket[1][1] > rest_ticket[1][1] + 0.025
        and open_ticket[0][2] > rest_ticket[0][2] + 0.010,
        details=f"rest={rest_ticket}, open={open_ticket}",
    )

    rest_door = ctx.part_element_world_aabb(service_door, elem="door_panel")
    with ctx.pose({service_hinge: 1.20}):
        open_door = ctx.part_element_world_aabb(service_door, elem="door_panel")
    ctx.check(
        "service door swings outward from the cabinet side on a vertical hinge",
        rest_door is not None
        and open_door is not None
        and open_door[1][0] > rest_door[1][0] + 0.12,
        details=f"rest={rest_door}, open={open_door}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
