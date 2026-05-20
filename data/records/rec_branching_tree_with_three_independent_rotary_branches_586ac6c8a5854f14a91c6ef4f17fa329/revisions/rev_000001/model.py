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


FRAME_HEIGHT = 0.82
FRAME_WIDTH = 0.15
FRAME_THICK = 0.032
RAIL_WIDTH = 0.028

INNER_GAP = 0.024
PLATE_THICKNESS = 0.006
BARREL_RADIUS = 0.0044
PIN_RADIUS = 0.0044
HOLE_CLEARANCE = 0.0

BRANCH_SPECS = {
    "lower_branch": {
        "joint": "frame_to_lower_branch",
        "support_z": 0.18,
        "support_y": 0.087,
        "pad_w": 0.064,
        "block_h": 0.070,
        "pad_x": 0.014,
        "plate_x": 0.028,
        "body_len": 0.155,
        "nose_len": 0.022,
        "root_w": 0.018,
        "root_h": 0.026,
        "tip_w": 0.015,
        "tip_h": 0.020,
        "mass": 0.62,
    },
    "middle_branch": {
        "joint": "frame_to_middle_branch",
        "support_z": 0.43,
        "support_y": -0.087,
        "pad_w": 0.072,
        "block_h": 0.082,
        "pad_x": 0.016,
        "plate_x": 0.032,
        "body_len": 0.235,
        "nose_len": 0.025,
        "root_w": 0.020,
        "root_h": 0.030,
        "tip_w": 0.016,
        "tip_h": 0.022,
        "mass": 0.78,
    },
    "upper_branch": {
        "joint": "frame_to_upper_branch",
        "support_z": 0.66,
        "support_y": 0.085,
        "pad_w": 0.058,
        "block_h": 0.066,
        "pad_x": 0.013,
        "plate_x": 0.026,
        "body_len": 0.132,
        "nose_len": 0.020,
        "root_w": 0.017,
        "root_h": 0.024,
        "tip_w": 0.014,
        "tip_h": 0.018,
        "mass": 0.54,
    },
}


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane(
        obj=cq.Solid.makeCylinder(
            radius,
            length,
            cq.Vector(center[0], center[1] - (length / 2.0), center[2]),
            cq.Vector(0.0, 1.0, 0.0),
        )
    )


def _support_pivot_x(spec: dict[str, float | str]) -> float:
    return ((FRAME_THICK / 2.0) - 0.004) + float(spec["pad_x"]) + float(spec["plate_x"]) + BARREL_RADIUS


def _build_support(spec: dict[str, float | str]) -> cq.Workplane:
    support_y = float(spec["support_y"])
    support_z = float(spec["support_z"])
    block_h = float(spec["block_h"])
    pad_x = float(spec["pad_x"])
    plate_x = float(spec["plate_x"])
    embed = 0.004
    total_y = INNER_GAP + (2.0 * PLATE_THICKNESS)
    full_depth = pad_x + plate_x
    rib_h = min(0.012, block_h * 0.22)
    open_h = block_h - (2.0 * rib_h)
    bracket_center_x = (FRAME_THICK / 2.0) + (full_depth / 2.0) - embed

    support = _box_at((full_depth, total_y, block_h), (bracket_center_x, support_y, support_z))
    support = support.cut(
        _box_at(
            (full_depth + 0.004, INNER_GAP, open_h),
            ((FRAME_THICK / 2.0) + (full_depth / 2.0), support_y, support_z),
        )
    )

    hole = _cylinder_y(
        PIN_RADIUS + HOLE_CLEARANCE,
        total_y + 0.010,
        (_support_pivot_x(spec), support_y, support_z),
    )
    return support.cut(hole)


def _build_frame_shape() -> cq.Shape:
    rail_y = (FRAME_WIDTH / 2.0) - (RAIL_WIDTH / 2.0)
    frame = _box_at((FRAME_THICK, RAIL_WIDTH, FRAME_HEIGHT), (0.0, rail_y, FRAME_HEIGHT / 2.0))
    frame = frame.union(
        _box_at((FRAME_THICK, RAIL_WIDTH, FRAME_HEIGHT), (0.0, -rail_y, FRAME_HEIGHT / 2.0))
    )

    rung_specs = (
        (0.052, 0.030, FRAME_WIDTH - 0.038, FRAME_THICK * 0.95),
        (0.245, 0.020, FRAME_WIDTH - 0.046, FRAME_THICK * 0.78),
        (0.445, 0.020, FRAME_WIDTH - 0.050, FRAME_THICK * 0.72),
        (0.635, 0.020, FRAME_WIDTH - 0.046, FRAME_THICK * 0.78),
        (0.768, 0.028, FRAME_WIDTH - 0.038, FRAME_THICK * 0.92),
    )
    for z_center, rung_h, rung_w, rung_x in rung_specs:
        frame = frame.union(_box_at((rung_x, rung_w, rung_h), (0.0, 0.0, z_center)))

    frame = frame.union(_box_at((0.018, 0.040, 0.58), (0.0, 0.0, 0.39)))

    for spec in BRANCH_SPECS.values():
        frame = frame.union(_build_support(spec))

    return frame.val()


def _build_branch_shape(spec: dict[str, float | str]) -> cq.Shape:
    body_len = float(spec["body_len"])
    nose_len = float(spec["nose_len"])
    root_w = float(spec["root_w"])
    root_h = float(spec["root_h"])
    tip_w = float(spec["tip_w"])
    tip_h = float(spec["tip_h"])
    arm_thickness = INNER_GAP - 0.006
    collar_len = 0.014
    collar_h = root_h * 0.86
    mid_x = collar_len + (body_len * 0.58)
    tip_x = collar_len + body_len
    nose_x = tip_x + nose_len

    barrel = _cylinder_y(BARREL_RADIUS, INNER_GAP, (0.0, 0.0, 0.0))
    collar = _box_at((collar_len, arm_thickness, collar_h), (collar_len / 2.0, 0.0, 0.0))
    beam = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.006, -(root_h * 0.42)),
                (mid_x, -(root_h * 0.26)),
                (tip_x, -(tip_h * 0.48)),
                (nose_x, 0.0),
                (tip_x, tip_h * 0.48),
                (mid_x, root_h * 0.26),
                (0.006, root_h * 0.42),
            ]
        )
        .close()
        .extrude(arm_thickness, both=True)
    )

    return barrel.union(collar).union(beam).val()


def _branch_inertial(spec: dict[str, float | str]) -> Inertial:
    body_len = float(spec["body_len"])
    nose_len = float(spec["nose_len"])
    root_w = float(spec["root_w"])
    root_h = float(spec["root_h"])
    mass = float(spec["mass"])
    approx_len = body_len + nose_len + 0.020
    return Inertial.from_geometry(
        Box((approx_len, root_w + 0.010, root_h + 0.010)),
        mass=mass,
        origin=Origin(xyz=(approx_len / 2.0, 0.0, 0.0)),
    )


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][2] + aabb[1][2]) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_fixture")

    model.material("frame_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("arm_aluminum", rgba=(0.70, 0.73, 0.77, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "fixture_frame"),
        material="frame_steel",
        name="frame_skin",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.090, FRAME_WIDTH, FRAME_HEIGHT)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    for branch_name, spec in BRANCH_SPECS.items():
        branch = model.part(branch_name)
        branch.visual(
            mesh_from_cadquery(_build_branch_shape(spec), f"{branch_name}_skin"),
            material="arm_aluminum",
            name="arm_skin",
        )
        branch.inertial = _branch_inertial(spec)

        model.articulation(
            str(spec["joint"]),
            ArticulationType.REVOLUTE,
            parent=frame,
            child=branch,
            origin=Origin(
                xyz=(
                    _support_pivot_x(spec),
                    float(spec["support_y"]),
                    float(spec["support_z"]),
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                lower=-0.35,
                upper=1.15,
                effort=18.0,
                velocity=1.6,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lower_branch = object_model.get_part("lower_branch")
    middle_branch = object_model.get_part("middle_branch")
    upper_branch = object_model.get_part("upper_branch")
    lower_joint = object_model.get_articulation("frame_to_lower_branch")
    middle_joint = object_model.get_articulation("frame_to_middle_branch")
    upper_joint = object_model.get_articulation("frame_to_upper_branch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    # The clevis-pin contacts land on a mesh-exact tolerance boundary in this model.
    # Use a tiny explicit contact tolerance so supported hinged branches are not
    # misclassified as floating due to 1e-6 numerical noise.
    ctx.fail_if_isolated_parts(contact_tol=2e-6)
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

    for branch in (lower_branch, middle_branch, upper_branch):
        ctx.expect_contact(
            branch,
            frame,
            contact_tol=2e-6,
            name=f"{branch.name}_mounted_to_frame",
        )
        ctx.expect_overlap(branch, frame, axes="yz", min_overlap=0.018, name=f"{branch.name}_supported_footprint")

    ctx.expect_origin_gap(upper_branch, middle_branch, axis="z", min_gap=0.18, max_gap=0.30, name="upper_branch_above_middle_branch")
    ctx.expect_origin_gap(middle_branch, lower_branch, axis="z", min_gap=0.18, max_gap=0.32, name="middle_branch_above_lower_branch")

    closed_fronts = {}
    for branch in (lower_branch, middle_branch, upper_branch):
        aabb = ctx.part_world_aabb(branch)
        closed_fronts[branch.name] = None if aabb is None else aabb[1][0]
    ctx.check(
        "branch_reach_lengths_are_staggered",
        (
            closed_fronts["middle_branch"] is not None
            and closed_fronts["lower_branch"] is not None
            and closed_fronts["upper_branch"] is not None
            and closed_fronts["middle_branch"] > (closed_fronts["lower_branch"] + 0.030)
            and closed_fronts["lower_branch"] > (closed_fronts["upper_branch"] + 0.015)
        ),
        details=(
            "Expected intentionally uneven forward reach with middle > lower > upper; "
            f"got {closed_fronts}"
        ),
    )

    for branch, joint in (
        (lower_branch, lower_joint),
        (middle_branch, middle_joint),
        (upper_branch, upper_joint),
    ):
        closed_aabb = ctx.part_element_world_aabb(branch, elem="arm_skin")
        pose_q = 0.85
        with ctx.pose({joint: pose_q}):
            raised_aabb = ctx.part_element_world_aabb(branch, elem="arm_skin")
        closed_z = _aabb_center_z(closed_aabb)
        raised_z = _aabb_center_z(raised_aabb)
        ctx.check(
            f"{branch.name}_positive_rotation_lifts_branch",
            (
                closed_z is not None
                and raised_z is not None
                and raised_z > (closed_z + 0.025)
            ),
            details=(
                f"Expected positive {joint.name} motion to raise {branch.name}; "
                f"closed_z={closed_z}, raised_z={raised_z}, q={pose_q}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
