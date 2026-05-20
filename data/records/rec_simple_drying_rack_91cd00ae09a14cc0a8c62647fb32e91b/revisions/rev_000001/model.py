from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


ROD = 0.011
HINGE_ROD = 0.014
RAIL = 0.0075


def _origin_between(p0, p1) -> tuple[Origin, float]:
    """Return a cylinder origin that points local +Z from p0 toward p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        raise ValueError("rod endpoints must be separated")
    nx, ny, nz = dx / length, dy / length, dz / length
    radial = sqrt(nx * nx + ny * ny)
    yaw = atan2(ny, nx) if radial > 1e-9 else 0.0
    pitch = atan2(radial, nz)
    center = (
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_rod(part, name: str, p0, p1, radius: float, material, overrun: float = 0.0) -> None:
    """Add one round tube, slightly over-running its nominal endpoints for welded joints."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    if overrun > 0.0:
        ux, uy, uz = dx / length, dy / length, dz / length
        p0 = (p0[0] - ux * overrun, p0[1] - uy * overrun, p0[2] - uz * overrun)
        p1 = (p1[0] + ux * overrun, p1[1] + uy * overrun, p1[2] + uz * overrun)
    origin, length = _origin_between(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    white_tube = model.material("white_powder_coated_tube", rgba=(0.92, 0.93, 0.90, 1.0))
    gray_hinge = model.material("warm_gray_hinges", rgba=(0.56, 0.58, 0.58, 1.0))
    dark_feet = model.material("dark_rubber_feet", rgba=(0.06, 0.065, 0.06, 1.0))

    top_z = 0.88
    central_len = 1.16
    central_w = 0.44
    half_len = central_len * 0.5
    half_w = central_w * 0.5

    central = model.part("central_frame")

    # Central rectangular drying surface: perimeter tube plus closely spaced hanging rails.
    _add_rod(central, "front_side_tube", (-half_len, half_w, top_z), (half_len, half_w, top_z), ROD, white_tube, ROD)
    _add_rod(central, "rear_side_tube", (-half_len, -half_w, top_z), (half_len, -half_w, top_z), ROD, white_tube, ROD)
    _add_rod(central, "end_tube_0", (-half_len, -half_w, top_z), (-half_len, half_w, top_z), ROD, white_tube, ROD)
    _add_rod(central, "end_tube_1", (half_len, -half_w, top_z), (half_len, half_w, top_z), ROD, white_tube, ROD)
    for idx, y in enumerate((-0.15, -0.075, 0.0, 0.075, 0.15)):
        _add_rod(
            central,
            f"center_hanging_rail_{idx}",
            (-half_len + 0.010, y, top_z + 0.002),
            (half_len - 0.010, y, top_z + 0.002),
            RAIL,
            white_tube,
            RAIL,
        )

    # Alternating hinge knuckles and short tabs make the wing hinge lines legible
    # without letting the two articulated parts occupy the same barrel segments.
    hinge_y = half_w + 0.028
    for side_name, sign in (("front", 1.0), ("rear", -1.0)):
        for idx, x in enumerate((-0.45, -0.09, 0.27)):
            _add_rod(
                central,
                f"{side_name}_hinge_knuckle_{idx}",
                (x - 0.087, sign * hinge_y, top_z - 0.028),
                (x + 0.087, sign * hinge_y, top_z - 0.028),
                HINGE_ROD,
                gray_hinge,
            )
            _add_rod(
                central,
                f"{side_name}_hinge_tab_{idx}",
                (x, sign * (half_w - 0.010), top_z - 0.004),
                (x, sign * hinge_y, top_z - 0.028),
                RAIL,
                gray_hinge,
                RAIL * 0.5,
            )

    # The lower folding support is carried from a separate underslung hinge.
    support_hinge_y = -0.17
    support_hinge_z = top_z - 0.070
    for idx, x in enumerate((-0.45, -0.09, 0.27)):
        _add_rod(
            central,
            f"support_hinge_lug_{idx}",
            (x - 0.090, support_hinge_y, support_hinge_z),
            (x + 0.090, support_hinge_y, support_hinge_z),
            HINGE_ROD,
            gray_hinge,
        )
        _add_rod(
            central,
            f"support_hinge_drop_{idx}",
            (x, -half_w, top_z),
            (x, support_hinge_y, support_hinge_z),
            RAIL,
            gray_hinge,
            RAIL * 0.5,
        )

    def add_wing(name: str, outward: float):
        wing = model.part(name)
        wing_len = 1.06
        wing_depth = 0.42
        x0, x1 = -wing_len * 0.5, wing_len * 0.5
        inner_y = outward * 0.034
        outer_y = outward * wing_depth

        _add_rod(wing, "inner_side_tube", (x0, inner_y, 0.0), (x1, inner_y, 0.0), ROD, white_tube, ROD)
        _add_rod(wing, "outer_side_tube", (x0, outer_y, 0.0), (x1, outer_y, 0.0), ROD, white_tube, ROD)
        _add_rod(wing, "end_tube_0", (x0, inner_y, 0.0), (x0, outer_y, 0.0), ROD, white_tube, ROD)
        _add_rod(wing, "end_tube_1", (x1, inner_y, 0.0), (x1, outer_y, 0.0), ROD, white_tube, ROD)
        for idx, y_mag in enumerate((0.105, 0.175, 0.245, 0.315)):
            _add_rod(
                wing,
                f"wing_hanging_rail_{idx}",
                (x0 + 0.010, outward * y_mag, 0.002),
                (x1 - 0.010, outward * y_mag, 0.002),
                RAIL,
                white_tube,
                RAIL,
            )
        for idx, x in enumerate((-0.27, 0.09, 0.45)):
            _add_rod(
                wing,
                f"hinge_knuckle_{idx}",
                (x - 0.096, 0.0, -0.028),
                (x + 0.096, 0.0, -0.028),
                HINGE_ROD,
                gray_hinge,
            )
            _add_rod(
                wing,
                f"hinge_link_{idx}",
                (x, 0.0, -0.028),
                (x, inner_y, 0.0),
                RAIL,
                gray_hinge,
                RAIL * 0.5,
            )
        return wing

    wing_0 = add_wing("wing_0", 1.0)
    wing_1 = add_wing("wing_1", -1.0)

    support = model.part("support_frame")
    # A folding lower frame: two side supports, one deliberately offset in depth
    # so the sloping profile remains visible from the side.
    support_width = 1.02
    sx0, sx1 = -support_width * 0.5, support_width * 0.5
    left_top = (sx0, 0.042, -0.028)
    left_foot = (sx0, 0.42, -0.82)
    right_top = (sx1, 0.095, -0.028)
    right_foot = (sx1, 0.475, -0.82)
    for idx, x in enumerate((-0.27, 0.09, 0.45)):
        _add_rod(
            support,
            f"hinge_tube_{idx}",
            (x - 0.090, 0.0, 0.0),
            (x + 0.090, 0.0, 0.0),
            ROD,
            gray_hinge,
        )
        _add_rod(
            support,
            f"hinge_drop_{idx}",
            (x, 0.0, 0.0),
            (x, 0.042, -0.028),
            RAIL,
            gray_hinge,
            RAIL * 0.5,
        )
    _add_rod(support, "top_carrier_tube", (sx0, 0.042, -0.028), (sx1, 0.042, -0.028), ROD, gray_hinge, ROD)
    _add_rod(support, "offset_top_link", (sx1, 0.042, -0.028), right_top, RAIL, gray_hinge, RAIL)
    _add_rod(support, "side_support_0", left_top, left_foot, ROD, white_tube, ROD)
    _add_rod(support, "offset_side_support", right_top, right_foot, ROD, white_tube, ROD)
    _add_rod(support, "floor_foot_rail", (sx0 - 0.045, 0.42, -0.82), (sx1 + 0.045, 0.475, -0.82), ROD, white_tube, ROD)
    for idx, t in enumerate((0.28, 0.52, 0.76)):
        p0 = (sx0, left_top[1] + (left_foot[1] - left_top[1]) * t, left_top[2] + (left_foot[2] - left_top[2]) * t)
        p1 = (sx1, right_top[1] + (right_foot[1] - right_top[1]) * t, right_top[2] + (right_foot[2] - right_top[2]) * t)
        _add_rod(support, f"lower_hanging_rail_{idx}", p0, p1, RAIL, white_tube, RAIL)
    for idx, (x, y) in enumerate(((sx0, 0.42), (sx1, 0.475))):
        support.visual(
            Cylinder(radius=0.026, length=0.026),
            origin=Origin(xyz=(x, y, -0.84), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_feet,
            name=f"rubber_foot_{idx}",
        )

    model.articulation(
        "wing_0_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(0.0, hinge_y, top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=8.0, velocity=1.5),
    )
    model.articulation(
        "wing_1_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(0.0, -hinge_y, top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=8.0, velocity=1.5),
    )
    model.articulation(
        "support_hinge",
        ArticulationType.REVOLUTE,
        parent=central,
        child=support,
        origin=Origin(xyz=(0.0, support_hinge_y, support_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.65, effort=12.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    support = object_model.get_part("support_frame")
    wing_0_hinge = object_model.get_articulation("wing_0_hinge")
    wing_1_hinge = object_model.get_articulation("wing_1_hinge")
    support_hinge = object_model.get_articulation("support_hinge")

    ctx.expect_overlap(wing_0, central, axes="x", min_overlap=0.85, name="front wing spans the central rack length")
    ctx.expect_overlap(wing_1, central, axes="x", min_overlap=0.85, name="rear wing spans the central rack length")
    ctx.expect_overlap(support, central, axes="x", min_overlap=0.85, name="lower support spans the central rack")

    rest_wing_0 = ctx.part_world_aabb(wing_0)
    rest_wing_1 = ctx.part_world_aabb(wing_1)
    rest_support = ctx.part_world_aabb(support)
    with ctx.pose({wing_0_hinge: 1.0, wing_1_hinge: 1.0, support_hinge: 0.55}):
        folded_wing_0 = ctx.part_world_aabb(wing_0)
        folded_wing_1 = ctx.part_world_aabb(wing_1)
        folded_support = ctx.part_world_aabb(support)
    ctx.check(
        "front wing folds upward",
        rest_wing_0 is not None and folded_wing_0 is not None and folded_wing_0[1][2] > rest_wing_0[1][2] + 0.20,
        details=f"rest={rest_wing_0}, folded={folded_wing_0}",
    )
    ctx.check(
        "rear wing folds upward",
        rest_wing_1 is not None and folded_wing_1 is not None and folded_wing_1[1][2] > rest_wing_1[1][2] + 0.20,
        details=f"rest={rest_wing_1}, folded={folded_wing_1}",
    )
    ctx.check(
        "support frame folds toward the rack",
        rest_support is not None and folded_support is not None and folded_support[0][2] > rest_support[0][2] + 0.10,
        details=f"rest={rest_support}, folded={folded_support}",
    )

    return ctx.report()


object_model = build_object_model()
