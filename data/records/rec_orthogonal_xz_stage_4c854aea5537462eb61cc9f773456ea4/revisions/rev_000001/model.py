from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.420
BASE_W = 0.180
BASE_T = 0.016
END_BLOCK_L = 0.030
END_BLOCK_H = 0.024

X_RAIL_Y = 0.052
X_RAIL_L = 0.320
X_RAIL_W = 0.020
X_RAIL_H = 0.010
X_RAIL_SADDLE_W = 0.032
X_RAIL_SADDLE_H = 0.010

CARR_L = 0.140
CARR_W = 0.160
CARR_DECK_T = 0.018
X_BEARING_W = 0.030
X_BEARING_H = 0.022
X_BEARING_EMBED = 0.003

GUIDE_W = 0.070
GUIDE_T = 0.046
GUIDE_H = 0.185
GUIDE_RIB_W = 0.012
GUIDE_POST_EMBED = 0.002
GUIDE_RIB_SIDE_EMBED = 0.001
GUIDE_RIB_BASE_EMBED = 0.001
GUIDE_RAIL_SPAN = 0.044
GUIDE_RAIL_W = 0.012
GUIDE_RAIL_T = 0.012
GUIDE_RAIL_H = 0.150
GUIDE_RAIL_Z0 = 0.022

Z_BLOCK_W = 0.090
Z_BLOCK_T = 0.032
Z_BLOCK_H = 0.065
NECK_W = 0.040
NECK_T = 0.050
NECK_H = 0.030
TOP_PAD_W = 0.112
TOP_PAD_T = 0.090
TOP_PAD_H = 0.012
TOP_PAD_CENTER_Y = -0.028

X_TRAVEL = 0.080
Z_TRAVEL = 0.075

X_STAGE_JOINT_Z = BASE_T + X_RAIL_SADDLE_H + X_RAIL_H + CARR_DECK_T + X_BEARING_H - X_BEARING_EMBED
Z_STAGE_JOINT_Y = -(GUIDE_T / 2.0 + GUIDE_RAIL_T + Z_BLOCK_T / 2.0)
Z_STAGE_JOINT_Z = 0.028


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _base_body_shape() -> cq.Workplane:
    body = _box((BASE_L, BASE_W, BASE_T), (0.0, 0.0, BASE_T / 2.0))

    for y_pos in (-X_RAIL_Y, X_RAIL_Y):
        body = body.union(
            _box(
                (X_RAIL_L + 0.040, X_RAIL_SADDLE_W, X_RAIL_SADDLE_H),
                (0.0, y_pos, BASE_T + X_RAIL_SADDLE_H / 2.0),
            )
        )

    for x_pos in (-BASE_L / 2.0 + END_BLOCK_L / 2.0, BASE_L / 2.0 - END_BLOCK_L / 2.0):
        body = body.union(
            _box(
                (END_BLOCK_L, BASE_W * 0.76, END_BLOCK_H),
                (x_pos, 0.0, BASE_T + END_BLOCK_H / 2.0),
            )
        )

    body = body.cut(
        _box(
            (BASE_L - 0.140, BASE_W * 0.30, BASE_T * 0.55),
            (0.0, 0.0, BASE_T - (BASE_T * 0.55) / 2.0),
        )
    )

    return body


def _carriage_body_shape() -> cq.Workplane:
    body = _box((CARR_L, CARR_W, CARR_DECK_T), (0.0, 0.0, -CARR_DECK_T / 2.0))

    body = body.cut(
        _box(
            (CARR_L * 0.52, CARR_W * 0.40, CARR_DECK_T * 0.50),
            (0.0, 0.0, -(CARR_DECK_T * 0.25)),
        )
    )

    for y_pos in (-X_RAIL_Y, X_RAIL_Y):
        body = body.union(
            _box(
                (CARR_L * 0.82, X_BEARING_W, X_BEARING_H),
                (0.0, y_pos, -(CARR_DECK_T + X_BEARING_H / 2.0) + X_BEARING_EMBED),
            )
        )

    body = body.union(_box((GUIDE_W, GUIDE_T, GUIDE_H), (0.0, 0.0, GUIDE_H / 2.0)))

    for x_pos in (-GUIDE_W / 2.0 + GUIDE_RIB_W / 2.0, GUIDE_W / 2.0 - GUIDE_RIB_W / 2.0):
        body = body.union(
            _box((GUIDE_RIB_W, GUIDE_T * 0.92, 0.085), (x_pos, 0.0, 0.0425))
        )

    body = body.cut(
        _box(
            (GUIDE_RAIL_SPAN - GUIDE_RAIL_W * 1.2, GUIDE_T * 0.55, GUIDE_H * 0.42),
            (0.0, -(GUIDE_T * 0.14), GUIDE_H * 0.50),
        )
    )

    return body.combine(clean=True)


def _z_sled_body_shape() -> cq.Workplane:
    body = _box((Z_BLOCK_W, Z_BLOCK_T, Z_BLOCK_H), (0.0, 0.0, Z_BLOCK_H / 2.0))
    body = body.union(_box((NECK_W, NECK_T, NECK_H), (0.0, -0.016, Z_BLOCK_H + NECK_H / 2.0)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_stage")

    model.material("base_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("machined_gray", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("pad_light", rgba=(0.83, 0.85, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "xz_stage_base_body"),
        material="base_dark",
        name="base_body",
    )
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(
            xyz=(0.0, -X_RAIL_Y, BASE_T + X_RAIL_SADDLE_H + X_RAIL_H / 2.0),
        ),
        material="rail_steel",
        name="x_rail_left",
    )
    base.visual(
        Box((X_RAIL_L, X_RAIL_W, X_RAIL_H)),
        origin=Origin(
            xyz=(0.0, X_RAIL_Y, BASE_T + X_RAIL_SADDLE_H + X_RAIL_H / 2.0),
        ),
        material="rail_steel",
        name="x_rail_right",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARR_L, CARR_W, CARR_DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -CARR_DECK_T / 2.0)),
        material="machined_gray",
        name="carriage_deck",
    )
    carriage.visual(
        Box((CARR_L * 0.82, X_BEARING_W, X_BEARING_H)),
        origin=Origin(
            xyz=(0.0, -X_RAIL_Y, -(CARR_DECK_T + X_BEARING_H / 2.0) + X_BEARING_EMBED),
        ),
        material="machined_gray",
        name="x_bearing_left",
    )
    carriage.visual(
        Box((CARR_L * 0.82, X_BEARING_W, X_BEARING_H)),
        origin=Origin(
            xyz=(0.0, X_RAIL_Y, -(CARR_DECK_T + X_BEARING_H / 2.0) + X_BEARING_EMBED),
        ),
        material="machined_gray",
        name="x_bearing_right",
    )
    carriage.visual(
        Box((GUIDE_W, GUIDE_T, GUIDE_H)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_H / 2.0 - GUIDE_POST_EMBED)),
        material="machined_gray",
        name="guide_column",
    )
    carriage.visual(
        Box((GUIDE_RIB_W, GUIDE_T * 0.92, 0.085)),
        origin=Origin(
            xyz=(
                -(GUIDE_W / 2.0 - GUIDE_RIB_W / 2.0 - GUIDE_RIB_SIDE_EMBED),
                0.0,
                0.0425 - GUIDE_RIB_BASE_EMBED,
            )
        ),
        material="machined_gray",
        name="guide_rib_left",
    )
    carriage.visual(
        Box((GUIDE_RIB_W, GUIDE_T * 0.92, 0.085)),
        origin=Origin(
            xyz=(
                GUIDE_W / 2.0 - GUIDE_RIB_W / 2.0 - GUIDE_RIB_SIDE_EMBED,
                0.0,
                0.0425 - GUIDE_RIB_BASE_EMBED,
            )
        ),
        material="machined_gray",
        name="guide_rib_right",
    )
    carriage.visual(
        Box((GUIDE_RAIL_W, GUIDE_RAIL_T, GUIDE_RAIL_H)),
        origin=Origin(
            xyz=(
                -GUIDE_RAIL_SPAN / 2.0,
                -(GUIDE_T / 2.0 + GUIDE_RAIL_T / 2.0),
                GUIDE_RAIL_Z0 + GUIDE_RAIL_H / 2.0,
            )
        ),
        material="rail_steel",
        name="z_guide_left",
    )
    carriage.visual(
        Box((GUIDE_RAIL_W, GUIDE_RAIL_T, GUIDE_RAIL_H)),
        origin=Origin(
            xyz=(
                GUIDE_RAIL_SPAN / 2.0,
                -(GUIDE_T / 2.0 + GUIDE_RAIL_T / 2.0),
                GUIDE_RAIL_Z0 + GUIDE_RAIL_H / 2.0,
            )
        ),
        material="rail_steel",
        name="z_guide_right",
    )

    z_sled = model.part("z_sled")
    z_sled.visual(
        mesh_from_cadquery(_z_sled_body_shape(), "xz_stage_z_sled_body"),
        material="machined_gray",
        name="z_sled_body",
    )
    z_sled.visual(
        Box((TOP_PAD_W, TOP_PAD_T, TOP_PAD_H)),
        origin=Origin(
            xyz=(0.0, TOP_PAD_CENTER_Y, Z_BLOCK_H + NECK_H + TOP_PAD_H / 2.0),
        ),
        material="pad_light",
        name="top_pad",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=450.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_z_sled",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=z_sled,
        origin=Origin(xyz=(0.0, Z_STAGE_JOINT_Y, Z_STAGE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=250.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    z_sled = object_model.get_part("z_sled")
    x_stage = object_model.get_articulation("base_to_carriage")
    z_stage = object_model.get_articulation("carriage_to_z_sled")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.check(
        "x_stage_axis_is_horizontal",
        tuple(x_stage.axis) == (1.0, 0.0, 0.0),
        f"expected X-stage axis (1, 0, 0), got {x_stage.axis}",
    )
    ctx.check(
        "z_stage_axis_is_vertical",
        tuple(z_stage.axis) == (0.0, 0.0, 1.0),
        f"expected Z-stage axis (0, 0, 1), got {z_stage.axis}",
    )

    with ctx.pose({x_stage: 0.0, z_stage: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_b="x_rail_left",
            contact_tol=1e-5,
            name="carriage_seats_on_left_x_rail",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_b="x_rail_right",
            contact_tol=1e-5,
            name="carriage_seats_on_right_x_rail",
        )
        ctx.expect_contact(
            z_sled,
            carriage,
            elem_b="z_guide_left",
            contact_tol=1e-5,
            name="z_sled_contacts_left_guide",
        )
        ctx.expect_contact(
            z_sled,
            carriage,
            elem_b="z_guide_right",
            contact_tol=1e-5,
            name="z_sled_contacts_right_guide",
        )
        ctx.expect_gap(
            z_sled,
            base,
            axis="z",
            positive_elem="top_pad",
            min_gap=0.14,
            name="top_pad_clears_base_in_home_pose",
        )

    with ctx.pose({x_stage: X_TRAVEL, z_stage: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_b="x_rail_left",
            contact_tol=1e-5,
            name="carriage_stays_on_left_x_rail_at_limit",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_b="x_rail_right",
            contact_tol=1e-5,
            name="carriage_stays_on_right_x_rail_at_limit",
        )

    with ctx.pose({x_stage: 0.030, z_stage: Z_TRAVEL}):
        ctx.expect_contact(
            z_sled,
            carriage,
            elem_b="z_guide_left",
            contact_tol=1e-5,
            name="z_sled_stays_on_left_guide_at_limit",
        )
        ctx.expect_contact(
            z_sled,
            carriage,
            elem_b="z_guide_right",
            contact_tol=1e-5,
            name="z_sled_stays_on_right_guide_at_limit",
        )
        ctx.expect_gap(
            z_sled,
            base,
            axis="z",
            positive_elem="top_pad",
            min_gap=0.21,
            name="top_pad_lifts_clear_of_base",
        )

    with ctx.pose({x_stage: 0.0, z_stage: 0.0}):
        carriage_home = ctx.part_world_position(carriage)
    with ctx.pose({x_stage: X_TRAVEL, z_stage: 0.0}):
        carriage_xmax = ctx.part_world_position(carriage)

    x_ok = (
        carriage_home is not None
        and carriage_xmax is not None
        and carriage_xmax[0] > carriage_home[0] + 0.07
        and abs(carriage_xmax[1] - carriage_home[1]) < 1e-6
        and abs(carriage_xmax[2] - carriage_home[2]) < 1e-6
    )
    ctx.check(
        "x_stage_moves_carriage_only_along_x",
        x_ok,
        f"home={carriage_home}, xmax={carriage_xmax}",
    )

    with ctx.pose({x_stage: 0.030, z_stage: 0.0}):
        z_home = ctx.part_world_position(z_sled)
    with ctx.pose({x_stage: 0.030, z_stage: Z_TRAVEL}):
        z_high = ctx.part_world_position(z_sled)

    z_ok = (
        z_home is not None
        and z_high is not None
        and z_high[2] > z_home[2] + 0.07
        and abs(z_high[0] - z_home[0]) < 1e-6
        and abs(z_high[1] - z_home[1]) < 1e-6
    )
    ctx.check(
        "z_stage_moves_sled_only_along_z",
        z_ok,
        f"home={z_home}, high={z_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
