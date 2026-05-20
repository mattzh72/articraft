from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

KNOB_PART_NAMES = [
    "knob_left_outer",
    "knob_left_inner",
    "knob_right_inner",
    "knob_right_outer",
]
KNOB_JOINT_NAMES = [
    "knob_left_outer_joint",
    "knob_left_inner_joint",
    "knob_right_inner_joint",
    "knob_right_outer_joint",
]


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        try:
            return Material(name=name, rgba=rgba)
        except TypeError:
            return Material(name, rgba)


def _merge_box(
    geometry: BoxGeometry | None,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> BoxGeometry:
    piece = BoxGeometry(size)
    piece.translate(*center)
    if geometry is None:
        return piece
    geometry.merge(piece)
    return geometry


def _build_rack_geometry(
    width: float,
    depth: float,
    bar: float,
    cross_bar_count: int,
) -> BoxGeometry:
    geometry: BoxGeometry | None = None
    side_y = (width - bar) * 0.5
    front_x = depth * 0.5 - bar * 0.5
    back_x = -depth * 0.5 + bar * 0.5

    geometry = _merge_box(geometry, (depth, bar, bar), (0.0, -side_y, 0.0))
    geometry = _merge_box(geometry, (depth, bar, bar), (0.0, side_y, 0.0))
    geometry = _merge_box(geometry, (bar, width, bar), (front_x, 0.0, 0.0))
    geometry = _merge_box(geometry, (bar, width, bar), (back_x, 0.0, 0.0))
    geometry = _merge_box(geometry, (bar, width, bar * 2.8), (front_x, 0.0, bar * 0.9))

    usable_width = width - bar * 4.0
    for index in range(cross_bar_count):
        t = 0.5 if cross_bar_count == 1 else index / (cross_bar_count - 1)
        y = -usable_width * 0.5 + t * usable_width
        geometry = _merge_box(
            geometry,
            (depth - bar * 2.0, bar * 0.7, bar * 0.7),
            (0.0, y, bar * 0.2),
        )

    return geometry


def _build_knob_mesh():
    profile = [
        (0.0, 0.0),
        (0.016, 0.0),
        (0.021, 0.004),
        (0.023, 0.012),
        (0.023, 0.029),
        (0.018, 0.035),
        (0.0, 0.035),
    ]
    geometry = LatheGeometry(profile, segments=40)
    geometry.rotate_y(math.pi * 0.5)
    return mesh_from_geometry(geometry, ASSETS.mesh_path("oven_knob.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conventional_oven", assets=ASSETS)

    materials = {
        "stainless": _make_material("stainless_steel", (0.74, 0.76, 0.78, 1.0)),
        "brushed": _make_material("brushed_metal", (0.67, 0.69, 0.71, 1.0)),
        "charcoal": _make_material("charcoal_enamel", (0.14, 0.15, 0.17, 1.0)),
        "matte_black": _make_material("matte_black", (0.10, 0.10, 0.11, 1.0)),
        "rubber": _make_material("rubber", (0.08, 0.08, 0.08, 1.0)),
        "chrome": _make_material("chrome_rack", (0.82, 0.83, 0.84, 1.0)),
        "glass": _make_material("smoked_glass", (0.15, 0.18, 0.20, 0.35)),
        "display": _make_material("display_glass", (0.08, 0.11, 0.14, 0.75)),
        "indicator": _make_material("indicator_silver", (0.90, 0.91, 0.92, 1.0)),
    }
    model.materials.extend(materials.values())

    knob_mesh = _build_knob_mesh()
    rack_mesh = mesh_from_geometry(
        _build_rack_geometry(width=0.618, depth=0.452, bar=0.006, cross_bar_count=6),
        ASSETS.mesh_path("oven_rack.obj"),
    )

    body = model.part("body")
    steel = materials["stainless"]
    black = materials["matte_black"]
    enamel = materials["charcoal"]
    chrome = materials["chrome"]

    body.visual(Box((0.60, 0.03, 0.82)), origin=Origin(xyz=(0.0, -0.365, 0.44)), material=steel)
    body.visual(Box((0.60, 0.03, 0.82)), origin=Origin(xyz=(0.0, 0.365, 0.44)), material=steel)
    body.visual(Box((0.60, 0.76, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.875)), material=steel)
    body.visual(Box((0.56, 0.70, 0.045)), origin=Origin(xyz=(-0.01, 0.0, 0.0325)), material=steel)
    body.visual(Box((0.03, 0.70, 0.80)), origin=Origin(xyz=(-0.285, 0.0, 0.435)), material=steel)

    body.visual(Box((0.06, 0.03, 0.62)), origin=Origin(xyz=(0.285, -0.365, 0.37)), material=steel)
    body.visual(Box((0.06, 0.03, 0.62)), origin=Origin(xyz=(0.285, 0.365, 0.37)), material=steel)
    body.visual(Box((0.06, 0.73, 0.11)), origin=Origin(xyz=(0.285, 0.0, 0.055)), material=steel)
    body.visual(Box((0.06, 0.73, 0.055)), origin=Origin(xyz=(0.285, 0.0, 0.6825)), material=steel)
    body.visual(Box((0.06, 0.73, 0.04)), origin=Origin(xyz=(0.285, 0.0, 0.73)), material=steel)
    body.visual(Box((0.06, 0.73, 0.125)), origin=Origin(xyz=(0.285, 0.0, 0.8125)), material=steel)
    body.visual(
        Box((0.015, 0.76, 0.03)),
        origin=Origin(xyz=(0.3075, 0.0, 0.885)),
        material=materials["brushed"],
    )

    body.visual(
        Box((0.505, 0.018, 0.55)), origin=Origin(xyz=(0.0025, -0.339, 0.39)), material=enamel
    )
    body.visual(
        Box((0.505, 0.018, 0.55)), origin=Origin(xyz=(0.0025, 0.339, 0.39)), material=enamel
    )
    body.visual(Box((0.505, 0.66, 0.018)), origin=Origin(xyz=(0.0025, 0.0, 0.116)), material=enamel)
    body.visual(Box((0.505, 0.66, 0.018)), origin=Origin(xyz=(0.0025, 0.0, 0.664)), material=enamel)
    body.visual(Box((0.018, 0.66, 0.55)), origin=Origin(xyz=(-0.246, 0.0, 0.39)), material=enamel)

    body.visual(Box((0.010, 0.012, 0.55)), origin=Origin(xyz=(0.252, -0.324, 0.39)), material=black)
    body.visual(Box((0.010, 0.012, 0.55)), origin=Origin(xyz=(0.252, 0.324, 0.39)), material=black)
    body.visual(Box((0.010, 0.64, 0.012)), origin=Origin(xyz=(0.252, 0.0, 0.121)), material=black)
    body.visual(Box((0.010, 0.64, 0.012)), origin=Origin(xyz=(0.252, 0.0, 0.659)), material=black)

    body.visual(
        Cylinder(radius=0.078, length=0.004),
        origin=Origin(xyz=(-0.236, 0.0, 0.405), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=black,
    )
    body.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.231, 0.0, 0.405), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=materials["brushed"],
    )
    body.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(xyz=(-0.231, 0.235, 0.58), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=materials["glass"],
    )

    for z in (0.276, 0.463):
        body.visual(
            Box((0.46, 0.026, 0.006)), origin=Origin(xyz=(0.005, -0.321, z)), material=enamel
        )
        body.visual(
            Box((0.46, 0.026, 0.006)), origin=Origin(xyz=(0.005, 0.321, z)), material=enamel
        )
        body.visual(rack_mesh, origin=Origin(xyz=(0.005, 0.0, z)), material=chrome)

    body.visual(Box((0.008, 0.145, 0.065)), origin=Origin(xyz=(0.308, 0.0, 0.815)), material=black)
    body.visual(
        Box((0.002, 0.118, 0.045)),
        origin=Origin(xyz=(0.312, 0.0, 0.815)),
        material=materials["display"],
    )

    for y in (-0.085, -0.04, 0.04, 0.085):
        body.visual(
            Box((0.006, 0.022, 0.014)), origin=Origin(xyz=(0.307, y, 0.775)), material=black
        )
    for y in (-0.18, -0.09, 0.0, 0.09, 0.18):
        body.visual(
            Box((0.003, 0.058, 0.004)), origin=Origin(xyz=(0.307, y, 0.732)), material=black
        )

    for y in (-0.27, -0.09, 0.09, 0.27):
        body.visual(
            Cylinder(radius=0.032, length=0.004),
            origin=Origin(xyz=(0.313, y, 0.822), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=black,
        )

    for x in (-0.26, 0.26):
        for y in (-0.31, 0.31):
            body.visual(
                Cylinder(radius=0.022, length=0.02),
                origin=Origin(xyz=(x, y, 0.01)),
                material=materials["rubber"],
            )

    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.76, 0.90)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    door = model.part("door")
    door.visual(Box((0.040, 0.72, 0.60)), origin=Origin(xyz=(0.020, 0.0, 0.30)), material=steel)
    door.visual(Box((0.004, 0.56, 0.38)), origin=Origin(xyz=(0.039, 0.0, 0.31)), material=black)
    door.visual(
        Box((0.006, 0.44, 0.26)), origin=Origin(xyz=(0.042, 0.0, 0.31)), material=materials["glass"]
    )
    door.visual(Box((0.024, 0.64, 0.48)), origin=Origin(xyz=(0.014, 0.0, 0.275)), material=enamel)
    door.visual(
        Box((0.004, 0.40, 0.22)), origin=Origin(xyz=(0.020, 0.0, 0.29)), material=materials["glass"]
    )
    door.visual(
        Box((0.004, 0.58, 0.018)),
        origin=Origin(xyz=(0.039, 0.0, 0.145)),
        material=materials["brushed"],
    )
    door.visual(
        Cylinder(radius=0.014, length=0.54),
        origin=Origin(xyz=(0.085, 0.0, 0.515), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=materials["brushed"],
    )
    door.visual(
        Cylinder(radius=0.010, length=0.06),
        origin=Origin(xyz=(0.055, -0.255, 0.515), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=materials["brushed"],
    )
    door.visual(
        Cylinder(radius=0.010, length=0.06),
        origin=Origin(xyz=(0.055, 0.255, 0.515), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=materials["brushed"],
    )
    door.inertial = Inertial.from_geometry(
        Box((0.045, 0.72, 0.60)),
        mass=9.0,
        origin=Origin(xyz=(0.0225, 0.0, 0.30)),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="door",
        origin=Origin(xyz=(0.316, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    knob_positions = (-0.27, -0.09, 0.09, 0.27)
    for part_name, joint_name, y in zip(KNOB_PART_NAMES, KNOB_JOINT_NAMES, knob_positions):
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=black,
        )
        knob.visual(knob_mesh, origin=Origin(xyz=(0.012, 0.0, 0.0)), material=materials["brushed"])
        knob.visual(
            Box((0.006, 0.004, 0.012)),
            origin=Origin(xyz=(0.046, 0.0, 0.010)),
            material=materials["indicator"],
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.054, 0.046, 0.046)),
            mass=0.12,
            origin=Origin(xyz=(0.027, 0.0, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent="body",
            child=part_name,
            origin=Origin(xyz=(0.316, y, 0.822)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.7, velocity=4.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.0035, overlap_volume_tol=0.0)
    ctx.expect_joint_motion_axis(
        "door_hinge", "door", world_axis="z", direction="negative", min_delta=0.12
    )
    ctx.expect_joint_motion_axis(
        "door_hinge", "door", world_axis="x", direction="positive", min_delta=0.18
    )

    def require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    def _coerce_vec3(value) -> tuple[float, float, float] | None:
        if value is None:
            return None
        if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
            return (float(value.x), float(value.y), float(value.z))
        try:
            items = tuple(float(v) for v in value)
        except TypeError:
            return None
        if len(items) == 3:
            return items
        return None

    def bounds(part_name: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        aabb = ctx.part_world_aabb(part_name)

        for lo_name, hi_name in (
            ("min", "max"),
            ("mins", "maxs"),
            ("lower", "upper"),
            ("minimum", "maximum"),
            ("xyz_min", "xyz_max"),
            ("min_corner", "max_corner"),
        ):
            lo = _coerce_vec3(getattr(aabb, lo_name, None))
            hi = _coerce_vec3(getattr(aabb, hi_name, None))
            if lo is not None and hi is not None:
                return lo, hi

        if all(
            hasattr(aabb, name) for name in ("min_x", "min_y", "min_z", "max_x", "max_y", "max_z")
        ):
            return (
                (float(aabb.min_x), float(aabb.min_y), float(aabb.min_z)),
                (float(aabb.max_x), float(aabb.max_y), float(aabb.max_z)),
            )

        if isinstance(aabb, dict):
            for lo_name, hi_name in (
                ("min", "max"),
                ("mins", "maxs"),
                ("lower", "upper"),
                ("minimum", "maximum"),
                ("xyz_min", "xyz_max"),
            ):
                lo = _coerce_vec3(aabb.get(lo_name))
                hi = _coerce_vec3(aabb.get(hi_name))
                if lo is not None and hi is not None:
                    return lo, hi

        try:
            items = tuple(aabb)
        except TypeError:
            items = ()

        if len(items) == 2:
            lo = _coerce_vec3(items[0])
            hi = _coerce_vec3(items[1])
            if lo is not None and hi is not None:
                return lo, hi
        if len(items) == 6:
            return (
                (float(items[0]), float(items[1]), float(items[2])),
                (float(items[3]), float(items[4]), float(items[5])),
            )

        if hasattr(aabb, "__dict__"):
            values = {key: type(value).__name__ for key, value in vars(aabb).items()}
            raise AssertionError(
                f"Unsupported AABB representation for {part_name!r}: {type(aabb).__name__} {values}"
            )
        raise AssertionError(
            f"Unsupported AABB representation for {part_name!r}: {type(aabb).__name__}"
        )

    def center(part_name: str) -> tuple[float, float, float]:
        mins, maxs = bounds(part_name)
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    def extents(part_name: str) -> tuple[float, float, float]:
        mins, maxs = bounds(part_name)
        return tuple(maxs[index] - mins[index] for index in range(3))

    body_mins, body_maxs = bounds("body")
    door_mins, door_maxs = bounds("door")
    body_size = extents("body")
    door_size = extents("door")

    require(0.60 <= body_size[0] <= 0.66, f"Body depth out of range: {body_size[0]:.3f}")
    require(0.74 <= body_size[1] <= 0.78, f"Body width out of range: {body_size[1]:.3f}")
    require(0.88 <= body_size[2] <= 0.92, f"Body height out of range: {body_size[2]:.3f}")

    require(0.035 <= door_size[0] <= 0.10, f"Door thickness out of range: {door_size[0]:.3f}")
    require(0.70 <= door_size[1] <= 0.74, f"Door width out of range: {door_size[1]:.3f}")
    require(0.59 <= door_size[2] <= 0.61, f"Door height out of range: {door_size[2]:.3f}")
    require(
        abs(center("door")[1] - center("body")[1]) < 0.01,
        "Door should stay centered on the oven face",
    )
    require(
        door_size[1] >= body_size[1] - 0.06, "Door should cover almost the full front opening width"
    )
    require(
        0.095 <= door_mins[2] <= 0.115,
        f"Door bottom should sit just above the floor, got {door_mins[2]:.3f}",
    )
    require(
        0.695 <= door_maxs[2] <= 0.715,
        f"Door top should sit below the control panel, got {door_maxs[2]:.3f}",
    )
    require(
        door_mins[0] >= body_maxs[0] - 0.02, "Closed door should sit flush with the cabinet front"
    )

    knob_rest_centers: dict[str, tuple[float, float, float]] = {}
    for knob_name in KNOB_PART_NAMES:
        knob_mins, knob_maxs = bounds(knob_name)
        knob_center = center(knob_name)
        knob_rest_centers[knob_name] = knob_center

        require(
            knob_center[1] > body_mins[1] + 0.05, f"{knob_name} should not sit on the cabinet edge"
        )
        require(
            knob_center[1] < body_maxs[1] - 0.05, f"{knob_name} should not sit on the cabinet edge"
        )
        require(
            knob_mins[2] > door_maxs[2] + 0.08,
            f"{knob_name} should clearly live in the control panel band",
        )
        require(
            knob_maxs[0] > body_maxs[0], f"{knob_name} should protrude in front of the cabinet face"
        )

    closed_door_center = center("door")
    with ctx.pose(door_hinge=1.45):
        open_mins, open_maxs = bounds("door")
        open_center = center("door")
        lowest_knob_z = min(bounds(knob_name)[0][2] for knob_name in KNOB_PART_NAMES)

        require(
            open_center[0] > closed_door_center[0] + 0.18,
            "Open door should swing forward substantially",
        )
        require(
            open_center[2] < closed_door_center[2] - 0.16, "Open door should drop downward clearly"
        )
        require(open_maxs[2] < lowest_knob_z - 0.04, "Open door should clear the control knobs")
        require(open_mins[2] > -0.02, "Open door should remain above the floor plane")

    with ctx.pose(
        knob_left_outer_joint=1.8,
        knob_left_inner_joint=-1.7,
        knob_right_inner_joint=1.2,
        knob_right_outer_joint=-2.0,
    ):
        for knob_name, rest_center in knob_rest_centers.items():
            moved_center = center(knob_name)
            max_center_shift = max(
                abs(moved_center[index] - rest_center[index]) for index in range(3)
            )
            require(
                max_center_shift < 0.012,
                f"{knob_name} should rotate in place, shifted {max_center_shift:.3f} m",
            )

            knob_mins, knob_maxs = bounds(knob_name)
            require(
                knob_maxs[0] > body_maxs[0],
                f"{knob_name} should remain proud of the panel when turned",
            )
            require(
                knob_mins[2] > door_maxs[2] + 0.08,
                f"{knob_name} should remain in the control band when turned",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
