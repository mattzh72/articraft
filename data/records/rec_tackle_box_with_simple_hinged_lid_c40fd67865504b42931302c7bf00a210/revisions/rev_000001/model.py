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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_LENGTH = 0.36
BODY_WIDTH = 0.22
BODY_HEIGHT = 0.12
WALL = 0.0035

LID_DEPTH = 0.028
LID_LENGTH = 0.372
LID_WIDTH = 0.236
LID_SIDE_CLEARANCE = 0.0045
LID_VERTICAL_CLEARANCE = 0.0008

HINGE_PIN_RADIUS = 0.0032
HINGE_OUTER_RADIUS = 0.0058
HINGE_AXIS_X = -(BODY_LENGTH * 0.5) - 0.004
HINGE_AXIS_Z = BODY_HEIGHT - 0.009
OUTER_KNUCKLE_LENGTH = 0.052
CENTER_KNUCKLE_LENGTH = 0.086
OUTER_KNUCKLE_Y = 0.074


def _tube_shell_mesh(name: str, *, length: float, outer_radius: float, inner_radius: float):
    tube = LatheGeometry.from_shell_profiles(
        [(outer_radius, -0.5 * length), (outer_radius, 0.5 * length)],
        [(inner_radius, -0.5 * length), (inner_radius, 0.5 * length)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    tube.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(tube, name)


def _closed_lid_panel_z() -> float:
    return BODY_HEIGHT + LID_VERTICAL_CLEARANCE + (WALL * 0.5)


def _lid_skirt_center_z_rel() -> float:
    underside_z_rel = _closed_lid_panel_z() - HINGE_AXIS_Z - (WALL * 0.5)
    return underside_z_rel - (LID_DEPTH * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    body_plastic = model.material("body_plastic", rgba=(0.20, 0.26, 0.18, 1.0))
    latch_plastic = model.material("latch_plastic", rgba=(0.15, 0.18, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.67, 0.71, 1.0))

    base = model.part("base_shell")
    base.visual(
        Box((BODY_LENGTH, BODY_WIDTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL * 0.5)),
        material=body_plastic,
        name="floor",
    )
    base.visual(
        Box((BODY_LENGTH - (2.0 * WALL), WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, (BODY_WIDTH * 0.5) - (WALL * 0.5), BODY_HEIGHT * 0.5)),
        material=body_plastic,
        name="left_wall",
    )
    base.visual(
        Box((BODY_LENGTH - (2.0 * WALL), WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -((BODY_WIDTH * 0.5) - (WALL * 0.5)), BODY_HEIGHT * 0.5)),
        material=body_plastic,
        name="right_wall",
    )
    base.visual(
        Box((WALL, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_LENGTH * 0.5) + (WALL * 0.5), 0.0, BODY_HEIGHT * 0.5)),
        material=body_plastic,
        name="rear_wall",
    )
    base.visual(
        Box((WALL, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=((BODY_LENGTH * 0.5) - (WALL * 0.5), 0.0, BODY_HEIGHT * 0.5)),
        material=body_plastic,
        name="front_wall",
    )
    for name, x_sign, y_sign in (
        ("foot_fl", 1.0, 1.0),
        ("foot_fr", 1.0, -1.0),
        ("foot_rl", -1.0, 1.0),
        ("foot_rr", -1.0, -1.0),
    ):
        base.visual(
            Box((0.032, 0.020, 0.004)),
            origin=Origin(
                xyz=(
                    x_sign * 0.118,
                    y_sign * 0.072,
                    0.002,
                )
            ),
            material=body_plastic,
            name=name,
        )

    for name, y_center in (("left_latch_boss", OUTER_KNUCKLE_Y), ("right_latch_boss", -OUTER_KNUCKLE_Y)):
        base.visual(
            Box((0.007, 0.030, 0.010)),
            origin=Origin(xyz=((BODY_LENGTH * 0.5), y_center, 0.101)),
            material=latch_plastic,
            name=name,
        )

    for name, y_center in (("left_hinge_knuckle", -OUTER_KNUCKLE_Y), ("right_hinge_knuckle", OUTER_KNUCKLE_Y)):
        base.visual(
            _tube_shell_mesh(
                f"{name}_mesh",
                length=OUTER_KNUCKLE_LENGTH,
                outer_radius=HINGE_OUTER_RADIUS,
                inner_radius=HINGE_PIN_RADIUS,
            ),
            origin=Origin(xyz=(HINGE_AXIS_X, y_center, HINGE_AXIS_Z)),
            material=body_plastic,
            name=name,
        )
        base.visual(
            Box((0.012, OUTER_KNUCKLE_LENGTH, 0.014)),
            origin=Origin(xyz=(HINGE_AXIS_X + 0.0015, y_center, HINGE_AXIS_Z - 0.005)),
            material=body_plastic,
            name=f"{name}_web",
        )

    lid = model.part("lid")
    lid_panel_center_z_rel = _closed_lid_panel_z() - HINGE_AXIS_Z
    lid.visual(
        Box((LID_LENGTH, LID_WIDTH, WALL)),
        origin=Origin(xyz=(LID_LENGTH * 0.5, 0.0, lid_panel_center_z_rel)),
        material=body_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((LID_LENGTH - (2.0 * WALL), WALL, LID_DEPTH)),
        origin=Origin(
            xyz=(
                LID_LENGTH * 0.5,
                (BODY_WIDTH * 0.5) + LID_SIDE_CLEARANCE + (WALL * 0.5),
                _lid_skirt_center_z_rel(),
            )
        ),
        material=body_plastic,
        name="left_skirt",
    )
    lid.visual(
        Box((LID_LENGTH - (2.0 * WALL), WALL, LID_DEPTH)),
        origin=Origin(
            xyz=(
                LID_LENGTH * 0.5,
                -((BODY_WIDTH * 0.5) + LID_SIDE_CLEARANCE + (WALL * 0.5)),
                _lid_skirt_center_z_rel(),
            )
        ),
        material=body_plastic,
        name="right_skirt",
    )
    lid.visual(
        Box((WALL, LID_WIDTH, LID_DEPTH)),
        origin=Origin(
            xyz=(
                LID_LENGTH - (WALL * 0.5),
                0.0,
                _lid_skirt_center_z_rel(),
            )
        ),
        material=body_plastic,
        name="front_skirt",
    )
    lid.visual(
        _tube_shell_mesh(
            "center_hinge_knuckle_mesh",
            length=CENTER_KNUCKLE_LENGTH,
            outer_radius=HINGE_OUTER_RADIUS,
            inner_radius=HINGE_PIN_RADIUS,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=body_plastic,
        name="center_hinge_knuckle",
    )
    lid.visual(
        Box((0.012, CENTER_KNUCKLE_LENGTH, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.004)),
        material=body_plastic,
        name="hinge_bridge",
    )
    for name, y_center in (("left_hook", OUTER_KNUCKLE_Y), ("right_hook", -OUTER_KNUCKLE_Y)):
        lid.visual(
            Box((0.005, 0.024, 0.006)),
            origin=Origin(
                xyz=(
                    LID_LENGTH - WALL - 0.0025,
                    y_center,
                    _lid_skirt_center_z_rel() - 0.010,
                )
            ),
            material=latch_plastic,
            name=name,
        )
    lid.visual(
        Box((0.012, 0.080, 0.010)),
        origin=Origin(
            xyz=(
                LID_LENGTH - (WALL * 0.5) + 0.006,
                0.0,
                _lid_skirt_center_z_rel() - 0.002,
            )
        ),
        material=latch_plastic,
        name="pull_tab",
    )

    hinge_pin = model.part("hinge_pin")
    hinge_pin.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=BODY_WIDTH - 0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pin_shaft",
    )
    hinge_pin.visual(
        Cylinder(radius=0.0056, length=0.004),
        origin=Origin(
            xyz=(0.0, -((BODY_WIDTH - 0.012) * 0.5) - 0.002, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pin_head",
    )
    hinge_pin.visual(
        Cylinder(radius=0.0044, length=0.003),
        origin=Origin(
            xyz=(0.0, ((BODY_WIDTH - 0.012) * 0.5) + 0.0015, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pin_peened_end",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "base_to_hinge_pin",
        ArticulationType.FIXED,
        parent=base,
        child=hinge_pin,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_shell")
    lid = object_model.get_part("lid")
    hinge_pin = object_model.get_part("hinge_pin")
    lid_hinge = object_model.get_articulation("base_to_lid")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        base,
        hinge_pin,
        reason=(
            "The steel hinge pin is intentionally modeled concentric with the molded rear hinge barrels. "
            "This overlap represents the assembled hinge hardware path that captures the lid on the base."
        ),
    )
    ctx.allow_overlap(
        hinge_pin,
        lid,
        reason=(
            "The lid rotates on the same captured hinge pin. This overlap represents the assembled pinned hinge "
            "rather than a free-standing rod beside the lid."
        ),
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        hinge_pin,
        base,
        elem_a="pin_shaft",
        elem_b="left_hinge_knuckle",
        contact_tol=0.0005,
        name="pin seats in left body knuckle",
    )
    ctx.expect_contact(
        hinge_pin,
        base,
        elem_a="pin_shaft",
        elem_b="right_hinge_knuckle",
        contact_tol=0.0005,
        name="pin seats in right body knuckle",
    )
    ctx.expect_contact(
        hinge_pin,
        lid,
        elem_a="pin_shaft",
        elem_b="center_hinge_knuckle",
        contact_tol=0.0005,
        name="pin seats in lid knuckle",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_panel",
            elem_b="floor",
            min_overlap=0.20,
            name="closed lid covers tray opening",
        )
        ctx.expect_contact(
            lid,
            base,
            elem_a="left_hook",
            elem_b="left_latch_boss",
            contact_tol=0.0006,
            name="left latch hook reaches catch boss",
        )
        ctx.expect_contact(
            lid,
            base,
            elem_a="right_hook",
            elem_b="right_latch_boss",
            contact_tol=0.0006,
            name="right latch hook reaches catch boss",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="floor",
            min_gap=0.115,
            max_gap=0.125,
            name="closed lid sits at realistic tray height",
        )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: math.radians(100.0)}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
        if closed_front is None or open_front is None:
            ctx.fail("front skirt measurable across lid motion", "missing front skirt AABB")
        else:
            ctx.check(
                "lid opens upward",
                open_front[0][2] > (closed_front[1][2] + 0.060),
                details=(
                    f"expected open front edge to rise above closed lid, "
                    f"got closed_top={closed_front[1][2]:.4f}, open_bottom={open_front[0][2]:.4f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
