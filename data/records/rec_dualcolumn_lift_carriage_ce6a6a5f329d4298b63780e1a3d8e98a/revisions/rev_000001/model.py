from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.140
BASE_D = 0.100
BASE_T = 0.014

CAN_FLANGE_R = 0.038
CAN_FLANGE_H = 0.006
CAN_R = 0.030
CAN_H = 0.044

PED_W = 0.052
PED_D = 0.040
PED_H = 0.012
PED_Z = BASE_T + CAN_FLANGE_H + CAN_H

POST_X = 0.037
POST_R = 0.0085
POST_H = 0.286

BRIDGE_D = 0.028
BRIDGE_T = 0.016
BRIDGE_Z = BASE_T + POST_H - BRIDGE_T

CARRIAGE_CENTER_W = 0.046
CARRIAGE_D = 0.048
CARRIAGE_BODY_H = 0.058
GUIDE_BLOCK_W = 0.028
GUIDE_BLOCK_D = 0.048
GUIDE_BLOCK_H = 0.019
GUIDE_BLOCK_Z = 0.021
BEARING_BORE_R = 0.0105
TABLE_SUPPORT_W = 0.064
TABLE_SUPPORT_D = 0.044
TABLE_SUPPORT_H = 0.010
TABLE_SUPPORT_Y = 0.016
TABLE_W = 0.095
TABLE_D = 0.060
TABLE_T = 0.008
TABLE_Y = 0.046

CARRIAGE_HOME_Z = PED_Z + PED_H + (CARRIAGE_BODY_H / 2.0)
LIFT_STROKE = 0.108


def _box(
    sx: float,
    sy: float,
    sz: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    centered: tuple[bool, bool, bool] = (True, True, False),
):
    return cq.Workplane("XY").box(sx, sy, sz, centered=centered).translate((x, y, z))


def _cylinder(radius: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z))


def _make_frame_core():
    base_plate = _box(BASE_W, BASE_D, BASE_T)
    can_flange = _cylinder(CAN_FLANGE_R, CAN_FLANGE_H, z=BASE_T)
    can_body = _cylinder(CAN_R, CAN_H, z=BASE_T + CAN_FLANGE_H)
    left_lower_collar = _cylinder(0.015, 0.010, x=-POST_X, z=BASE_T)
    right_lower_collar = _cylinder(0.015, 0.010, x=POST_X, z=BASE_T)

    return (
        base_plate.union(can_flange)
        .union(can_body)
        .union(left_lower_collar)
        .union(right_lower_collar)
    )


def _make_top_bridge():
    bridge_bar = _box(POST_X * 2.0 + 0.020, BRIDGE_D, BRIDGE_T, z=BRIDGE_Z)
    left_boss = _cylinder(0.016, BRIDGE_T, x=-POST_X, z=BRIDGE_Z)
    right_boss = _cylinder(0.016, BRIDGE_T, x=POST_X, z=BRIDGE_Z)
    underside_relief = _box(0.046, BRIDGE_D + 0.004, 0.009, z=BRIDGE_Z)

    return bridge_bar.union(left_boss).union(right_boss).cut(underside_relief)


def _make_posts():
    left_post = _cylinder(POST_R, POST_H, x=-POST_X, z=BASE_T)
    right_post = _cylinder(POST_R, POST_H, x=POST_X, z=BASE_T)
    return left_post.union(right_post)


def _make_carriage_body():
    central_block = cq.Workplane("XY").box(
        CARRIAGE_CENTER_W,
        CARRIAGE_D,
        CARRIAGE_BODY_H,
        centered=(True, True, True),
    )

    body = central_block
    for post_x in (-POST_X, POST_X):
        body = body.union(
            cq.Workplane("XY").box(
                GUIDE_BLOCK_W,
                GUIDE_BLOCK_D,
                GUIDE_BLOCK_H,
                centered=(True, True, True),
            ).translate((post_x, 0.0, -GUIDE_BLOCK_Z))
        )
        body = body.union(
            cq.Workplane("XY").box(
                GUIDE_BLOCK_W,
                GUIDE_BLOCK_D,
                GUIDE_BLOCK_H,
                centered=(True, True, True),
            ).translate((post_x, 0.0, GUIDE_BLOCK_Z))
        )

    top_support = _box(
        TABLE_SUPPORT_W,
        TABLE_SUPPORT_D,
        TABLE_SUPPORT_H,
        y=TABLE_SUPPORT_Y,
        z=CARRIAGE_BODY_H / 2.0,
    )
    body = body.union(top_support)

    bore_height = CARRIAGE_BODY_H + TABLE_SUPPORT_H + 0.020
    for post_x in (-POST_X, POST_X):
        bore = _cylinder(
            BEARING_BORE_R,
            bore_height,
            x=post_x,
            z=-(bore_height / 2.0),
        )
        body = body.cut(bore)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cleanroom_lift_axis")

    model.material("enclosure_white", rgba=(0.95, 0.96, 0.97, 1.0))
    model.material("metal_post", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("carriage_gray", rgba=(0.73, 0.76, 0.79, 1.0))
    model.material("table_white", rgba=(0.98, 0.98, 0.99, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_core(), "frame_core"),
        material="enclosure_white",
        name="base_core",
    )
    frame.visual(
        mesh_from_cadquery(_make_top_bridge(), "frame_bridge"),
        material="enclosure_white",
        name="bridge",
    )
    frame.visual(
        mesh_from_cadquery(_make_posts(), "frame_posts"),
        material="metal_post",
        name="posts",
    )
    frame.visual(
        Box((PED_W, PED_D, PED_H)),
        origin=Origin(xyz=(0.0, 0.0, PED_Z + (PED_H / 2.0))),
        material="enclosure_white",
        name="pedestal",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, POST_H + BASE_T)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (POST_H + BASE_T) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_body(), "carriage_body"),
        material="carriage_gray",
        name="carriage_body",
    )
    carriage.visual(
        Box((TABLE_W, TABLE_D, TABLE_T)),
        origin=Origin(
            xyz=(
                0.0,
                TABLE_Y,
                (CARRIAGE_BODY_H / 2.0) + TABLE_SUPPORT_H + (TABLE_T / 2.0),
            )
        ),
        material="table_white",
        name="table",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((TABLE_W, TABLE_D, 0.076)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.024, 0.009)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=LIFT_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")

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

    ctx.check("frame part exists", frame is not None)
    ctx.check("carriage part exists", carriage is not None)
    ctx.check(
        "lift is a vertical prismatic axis",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper == LIFT_STROKE,
        details=(
            f"type={lift.articulation_type}, axis={lift.axis}, "
            f"limits={lift.motion_limits}"
        ),
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_b="pedestal",
            name="carriage seats on the pedestal at home",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="xy",
            elem_b="posts",
            min_overlap=0.015,
            name="carriage stays centered on the guide posts at home",
        )

    with ctx.pose({lift: LIFT_STROKE}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="carriage_body",
            negative_elem="pedestal",
            min_gap=0.090,
            name="carriage lifts clear of the pedestal",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="bridge",
            negative_elem="table",
            min_gap=0.015,
            name="table remains below the top bridge at full lift",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="xy",
            elem_b="posts",
            min_overlap=0.015,
            name="carriage remains guided by both posts at full lift",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
