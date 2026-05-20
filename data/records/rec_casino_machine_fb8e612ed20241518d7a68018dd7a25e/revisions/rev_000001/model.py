from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="video_poker_cabinet")

    cabinet_shell = model.material("cabinet_shell", rgba=(0.17, 0.18, 0.20, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.08, 0.09, 0.10, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.12, 0.20, 0.24, 0.45))
    brushed_trim = model.material("brushed_trim", rgba=(0.53, 0.56, 0.60, 1.0))
    panel_blue = model.material("panel_blue", rgba=(0.18, 0.40, 0.72, 1.0))
    panel_green = model.material("panel_green", rgba=(0.18, 0.63, 0.28, 1.0))
    recess_dark = model.material("recess_dark", rgba=(0.03, 0.03, 0.04, 1.0))
    flap_dark = model.material("flap_dark", rgba=(0.11, 0.12, 0.13, 1.0))

    cabinet = model.part("cabinet")

    cabinet_width = 0.62
    cabinet_profile = [
        (-0.27, 0.00),
        (-0.27, 1.18),
        (-0.20, 1.28),
        (-0.05, 1.34),
        (0.09, 1.28),
        (0.20, 1.08),
        (0.28, 0.82),
        (0.29, 0.74),
        (0.29, 0.18),
        (0.22, 0.10),
        (0.19, 0.00),
    ]
    cabinet_geom = ExtrudeGeometry.from_z0(cabinet_profile, cabinet_width, cap=True, closed=True)
    cabinet.visual(
        mesh_from_geometry(cabinet_geom, "video_poker_cabinet_shell"),
        origin=Origin(xyz=(-cabinet_width / 2.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=cabinet_shell,
        name="cabinet_shell",
    )

    display_angle = math.radians(18.0)
    display_anchor = (0.0, 0.142, 1.045)
    deck_angle = math.radians(16.0)
    deck_anchor = (0.0, 0.155, 0.742)

    def rolled_origin(
        anchor: tuple[float, float, float],
        angle: float,
        local_xyz: tuple[float, float, float],
    ) -> Origin:
        local_x, local_y, local_z = local_xyz
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return Origin(
            xyz=(
                anchor[0] + local_x,
                anchor[1] + local_y * cos_a - local_z * sin_a,
                anchor[2] + local_y * sin_a + local_z * cos_a,
            ),
            rpy=(angle, 0.0, 0.0),
        )

    cabinet.visual(
        Box((0.50, 0.020, 0.58)),
        origin=Origin(xyz=(0.0, 0.300, 0.42)),
        material=fascia_black,
        name="lower_front_panel",
    )
    cabinet.visual(
        Box((0.54, 0.026, 0.10)),
        origin=Origin(xyz=(0.0, 0.294, 0.09)),
        material=fascia_black,
        name="kick_panel",
    )
    cabinet.visual(
        Box((0.52, 0.16, 0.54)),
        origin=Origin(xyz=(0.0, 0.225, 0.34)),
        material=cabinet_shell,
        name="internal_fascia_mass",
    )
    cabinet.visual(
        Box((0.50, 0.14, 0.34)),
        origin=Origin(xyz=(0.0, 0.160, 0.96)),
        material=cabinet_shell,
        name="upper_display_mass",
    )
    cabinet.visual(
        Box((0.54, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.205, 0.67)),
        material=cabinet_shell,
        name="deck_support_mass",
    )

    cabinet.visual(
        Box((0.47, 0.045, 0.35)),
        origin=rolled_origin(display_anchor, display_angle, (0.0, 0.0, 0.0)),
        material=fascia_black,
        name="display_bezel",
    )
    cabinet.visual(
        Box((0.39, 0.008, 0.25)),
        origin=rolled_origin(display_anchor, display_angle, (0.0, 0.019, 0.0)),
        material=smoked_glass,
        name="screen_glass",
    )
    cabinet.visual(
        Box((0.47, 0.010, 0.030)),
        origin=rolled_origin(display_anchor, display_angle, (0.0, 0.025, -0.18)),
        material=brushed_trim,
        name="display_lower_trim",
    )

    cabinet.visual(
        Box((0.56, 0.22, 0.028)),
        origin=rolled_origin(deck_anchor, deck_angle, (0.0, 0.0, -0.010)),
        material=fascia_black,
        name="control_deck",
    )
    cabinet.visual(
        Box((0.46, 0.055, 0.010)),
        origin=rolled_origin(deck_anchor, deck_angle, (0.0, -0.012, 0.004)),
        material=brushed_trim,
        name="button_row_base",
    )

    card_button_centers = (-0.176, -0.088, 0.0, 0.088, 0.176)
    for index, center_x in enumerate(card_button_centers, start=1):
        cabinet.visual(
            Box((0.068, 0.036, 0.016)),
            origin=rolled_origin(deck_anchor, deck_angle, (center_x, -0.012, 0.011)),
            material=panel_blue,
            name=f"card_button_{index}",
        )

    well_center_x = 0.242
    well_center_y = 0.034
    well_opening_x = 0.090
    well_opening_y = 0.048
    well_wall = 0.006
    well_height = 0.0045

    cabinet.visual(
        Box((well_wall, well_opening_y + 2.0 * well_wall, well_height)),
        origin=rolled_origin(
            deck_anchor,
            deck_angle,
            (well_center_x - well_opening_x / 2.0 - well_wall / 2.0, well_center_y, well_height / 2.0),
        ),
        material=brushed_trim,
        name="cashout_well_left",
    )
    cabinet.visual(
        Box((well_wall, well_opening_y + 2.0 * well_wall, well_height)),
        origin=rolled_origin(
            deck_anchor,
            deck_angle,
            (well_center_x + well_opening_x / 2.0 + well_wall / 2.0, well_center_y, well_height / 2.0),
        ),
        material=brushed_trim,
        name="cashout_well_right",
    )
    cabinet.visual(
        Box((well_opening_x, well_wall, well_height)),
        origin=rolled_origin(
            deck_anchor,
            deck_angle,
            (well_center_x, well_center_y - well_opening_y / 2.0 - well_wall / 2.0, well_height / 2.0),
        ),
        material=brushed_trim,
        name="cashout_well_front",
    )
    cabinet.visual(
        Box((well_opening_x, well_wall, well_height)),
        origin=rolled_origin(
            deck_anchor,
            deck_angle,
            (well_center_x, well_center_y + well_opening_y / 2.0 + well_wall / 2.0, well_height / 2.0),
        ),
        material=brushed_trim,
        name="cashout_well_back",
    )
    cabinet.visual(
        Box((well_opening_x, well_opening_y, 0.0012)),
        origin=rolled_origin(deck_anchor, deck_angle, (well_center_x, well_center_y, 0.0002)),
        material=recess_dark,
        name="cashout_well_floor",
    )

    cabinet.visual(
        Box((0.19, 0.022, 0.10)),
        origin=Origin(xyz=(0.0, 0.307, 0.49)),
        material=brushed_trim,
        name="bill_acceptor_bezel",
    )
    cabinet.visual(
        Box((0.116, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.307, 0.495)),
        material=recess_dark,
        name="bill_acceptor_mouth",
    )
    cabinet.visual(
        Box((0.130, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.315, 0.515)),
        material=fascia_black,
        name="bill_acceptor_hinge_cover",
    )

    cabinet.visual(
        Box((0.24, 0.040, 0.11)),
        origin=Origin(xyz=(0.0, 0.313, 0.29)),
        material=brushed_trim,
        name="payout_tray_bezel",
    )
    cabinet.visual(
        Box((0.186, 0.024, 0.046)),
        origin=Origin(xyz=(0.0, 0.320, 0.285)),
        material=recess_dark,
        name="payout_tray_cavity",
    )
    cabinet.visual(
        Box((0.170, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.322, 0.255)),
        material=fascia_black,
        name="payout_tray_floor",
    )
    cabinet.visual(
        Box((0.196, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.330, 0.333)),
        material=fascia_black,
        name="payout_tray_hinge_cover",
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((0.62, 0.56, 1.34)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.01, 0.67)),
    )

    bill_flap = model.part("bill_acceptor_flap")
    bill_flap.visual(
        Box((0.118, 0.006, 0.006)),
        origin=Origin(),
        material=flap_dark,
        name="bill_flap_hinge",
    )
    bill_flap.visual(
        Box((0.116, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=flap_dark,
        name="bill_flap_panel",
    )
    bill_flap.inertial = Inertial.from_geometry(
        Box((0.118, 0.008, 0.036)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    payout_flap = model.part("payout_tray_flap")
    payout_flap.visual(
        Box((0.180, 0.008, 0.008)),
        origin=Origin(),
        material=flap_dark,
        name="payout_flap_hinge",
    )
    payout_flap.visual(
        Box((0.180, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=flap_dark,
        name="payout_flap_panel",
    )
    payout_flap.visual(
        Box((0.180, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, -0.055)),
        material=flap_dark,
        name="payout_flap_lip",
    )
    payout_flap.inertial = Inertial.from_geometry(
        Box((0.182, 0.018, 0.065)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.004, -0.032)),
    )

    cash_out_button = model.part("cash_out_button")
    cash_out_button.visual(
        Box((0.082, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=panel_green,
        name="cashout_cap",
    )
    cash_out_button.inertial = Inertial.from_geometry(
        Box((0.082, 0.040, 0.012)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "cabinet_to_bill_acceptor_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=bill_flap,
        origin=Origin(xyz=(0.0, 0.322, 0.513)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=0.70,
        ),
    )
    model.articulation(
        "cabinet_to_payout_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=payout_flap,
        origin=Origin(xyz=(0.0, 0.338, 0.333)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "cabinet_to_cash_out_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=cash_out_button,
        origin=rolled_origin(deck_anchor, deck_angle, (well_center_x, well_center_y, well_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.08,
            lower=-0.004,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    bill_flap = object_model.get_part("bill_acceptor_flap")
    payout_flap = object_model.get_part("payout_tray_flap")
    cash_out_button = object_model.get_part("cash_out_button")

    bill_joint = object_model.get_articulation("cabinet_to_bill_acceptor_flap")
    payout_joint = object_model.get_articulation("cabinet_to_payout_tray_flap")
    cash_joint = object_model.get_articulation("cabinet_to_cash_out_button")

    bill_bezel = cabinet.get_visual("bill_acceptor_bezel")
    tray_bezel = cabinet.get_visual("payout_tray_bezel")
    control_deck = cabinet.get_visual("control_deck")
    cashout_well_floor = cabinet.get_visual("cashout_well_floor")
    bill_panel = bill_flap.get_visual("bill_flap_panel")
    payout_panel = payout_flap.get_visual("payout_flap_panel")
    cash_cap = cash_out_button.get_visual("cashout_cap")

    for index in range(1, 6):
        cabinet.get_visual(f"card_button_{index}")

    with ctx.pose({bill_joint: 0.0, payout_joint: 0.0, cash_joint: 0.0}):
        ctx.expect_gap(
            bill_flap,
            cabinet,
            axis="y",
            min_gap=0.0,
            max_gap=0.0025,
            positive_elem=bill_panel,
            negative_elem=bill_bezel,
            name="bill acceptor flap seats at the bezel mouth",
        )
        ctx.expect_overlap(
            bill_flap,
            cabinet,
            axes="xz",
            min_overlap=0.03,
            elem_a=bill_panel,
            elem_b=bill_bezel,
            name="bill acceptor flap spans the acceptor opening",
        )
        ctx.expect_gap(
            payout_flap,
            cabinet,
            axis="y",
            min_gap=0.0,
            max_gap=0.0025,
            positive_elem=payout_panel,
            negative_elem=tray_bezel,
            name="payout tray flap closes against the tray bezel",
        )
        ctx.expect_overlap(
            payout_flap,
            cabinet,
            axes="xz",
            min_overlap=0.05,
            elem_a=payout_panel,
            elem_b=tray_bezel,
            name="payout tray flap covers the tray opening",
        )
        ctx.expect_overlap(
            cash_out_button,
            cabinet,
            axes="xy",
            min_overlap=0.03,
            elem_a=cash_cap,
            elem_b=control_deck,
            name="cash-out button sits over the control deck footprint",
        )

    bill_rest = ctx.part_element_world_aabb(bill_flap, elem=bill_panel)
    with ctx.pose({bill_joint: bill_joint.motion_limits.upper}):
        bill_open = ctx.part_element_world_aabb(bill_flap, elem=bill_panel)
    ctx.check(
        "bill acceptor flap opens outward from its mouth hinge",
        bill_rest is not None
        and bill_open is not None
        and bill_open[1][1] > bill_rest[1][1] + 0.015,
        details=f"rest={bill_rest}, open={bill_open}",
    )

    payout_rest = ctx.part_element_world_aabb(payout_flap, elem=payout_panel)
    with ctx.pose({payout_joint: payout_joint.motion_limits.upper}):
        payout_open = ctx.part_element_world_aabb(payout_flap, elem=payout_panel)
    ctx.check(
        "payout tray flap swings outward from the tray opening",
        payout_rest is not None
        and payout_open is not None
        and payout_open[1][1] > payout_rest[1][1] + 0.030,
        details=f"rest={payout_rest}, open={payout_open}",
    )

    cash_rest = ctx.part_world_position(cash_out_button)
    with ctx.pose({cash_joint: cash_joint.motion_limits.lower}):
        cash_pressed = ctx.part_world_position(cash_out_button)
        ctx.expect_overlap(
            cash_out_button,
            cabinet,
            axes="xy",
            min_overlap=0.02,
            elem_a=cash_cap,
            elem_b=cashout_well_floor,
            name="cash-out button stays centered over its well when pressed",
        )
    ctx.check(
        "cash-out button translates into the deck when pressed",
        cash_rest is not None
        and cash_pressed is not None
        and cash_pressed[1] > cash_rest[1] + 0.0008
        and cash_pressed[2] < cash_rest[2] - 0.0030,
        details=f"rest={cash_rest}, pressed={cash_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
