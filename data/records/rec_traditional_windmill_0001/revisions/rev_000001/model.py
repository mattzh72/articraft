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
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]):
    attempts = (
        lambda: Material(name=name, color=rgba),
        lambda: Material(name=name, rgba=rgba),
        lambda: Material(name, rgba),
        lambda: Material(name=name),
    )
    for attempt in attempts:
        try:
            return attempt()
        except TypeError:
            continue
    return None


def _materials():
    return {
        "stone": _make_material("stone", (0.67, 0.66, 0.62, 1.0)),
        "timber": _make_material("timber", (0.45, 0.31, 0.18, 1.0)),
        "timber_dark": _make_material("timber_dark", (0.25, 0.18, 0.11, 1.0)),
        "roof": _make_material("roof", (0.23, 0.17, 0.13, 1.0)),
        "iron": _make_material("iron", (0.18, 0.19, 0.20, 1.0)),
        "canvas": _make_material("canvas", (0.90, 0.88, 0.80, 1.0)),
    }


def _visual(part, geometry, *, origin=None, material=None, name=None):
    kwargs = {}
    if origin is not None:
        kwargs["origin"] = origin
    if material is not None:
        kwargs["material"] = material
    if name is not None:
        kwargs["name"] = name
    part.visual(geometry, **kwargs)


def _extract_xyz(value) -> tuple[float, float, float]:
    if isinstance(value, (tuple, list)) and len(value) == 3:
        return (float(value[0]), float(value[1]), float(value[2]))
    for attr_set in (("x", "y", "z"), ("px", "py", "pz")):
        if all(hasattr(value, attr) for attr in attr_set):
            return tuple(float(getattr(value, attr)) for attr in attr_set)
    raise TypeError(f"Unsupported position value: {value!r}")


def _extract_aabb_bounds(aabb) -> tuple[float, float, float, float, float, float]:
    if (
        isinstance(aabb, (tuple, list))
        and len(aabb) == 2
        and isinstance(aabb[0], (tuple, list))
        and isinstance(aabb[1], (tuple, list))
        and len(aabb[0]) == 3
        and len(aabb[1]) == 3
    ):
        min_pt = _extract_xyz(aabb[0])
        max_pt = _extract_xyz(aabb[1])
        return (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2])
    if all(hasattr(aabb, attr) for attr in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")):
        return (
            float(aabb.min_x),
            float(aabb.max_x),
            float(aabb.min_y),
            float(aabb.max_y),
            float(aabb.min_z),
            float(aabb.max_z),
        )
    if all(hasattr(aabb, attr) for attr in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")):
        return (
            float(aabb.xmin),
            float(aabb.xmax),
            float(aabb.ymin),
            float(aabb.ymax),
            float(aabb.zmin),
            float(aabb.zmax),
        )
    if hasattr(aabb, "min") and hasattr(aabb, "max"):
        min_pt = _extract_xyz(aabb.min)
        max_pt = _extract_xyz(aabb.max)
        return (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2])
    if hasattr(aabb, "lower") and hasattr(aabb, "upper"):
        min_pt = _extract_xyz(aabb.lower)
        max_pt = _extract_xyz(aabb.upper)
        return (min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2])
    raise TypeError(f"Unsupported AABB value: {aabb!r}")


def _add_horizontal_sail(part, sign: float, timber, canvas) -> None:
    root_x = sign * 0.36
    tip_x = sign * 1.60
    center_x = sign * ((abs(root_x) + abs(tip_x)) * 0.5)
    span = abs(tip_x - root_x)

    _visual(
        part,
        Box((span, 0.035, 0.035)),
        origin=Origin(xyz=(center_x, 0.055, 0.19)),
        material=timber,
    )
    _visual(
        part,
        Box((span, 0.035, 0.035)),
        origin=Origin(xyz=(center_x, 0.055, -0.19)),
        material=timber,
    )
    _visual(
        part,
        Box((0.04, 0.035, 0.42)),
        origin=Origin(xyz=(root_x, 0.055, 0.0)),
        material=timber,
    )
    _visual(
        part,
        Box((0.04, 0.035, 0.42)),
        origin=Origin(xyz=(tip_x, 0.055, 0.0)),
        material=timber,
    )

    for frac in (0.18, 0.37, 0.56, 0.75):
        slat_x = root_x + sign * (span * frac)
        _visual(
            part,
            Box((0.018, 0.018, 0.34)),
            origin=Origin(xyz=(slat_x, 0.06, 0.0)),
            material=timber,
        )

    for z in (-0.11, 0.0, 0.11):
        _visual(
            part,
            Box((span - 0.08, 0.006, 0.09)),
            origin=Origin(xyz=(center_x, 0.067, z)),
            material=canvas,
        )


def _add_vertical_sail(part, sign: float, timber, canvas) -> None:
    root_z = sign * 0.36
    tip_z = sign * 1.60
    center_z = sign * ((abs(root_z) + abs(tip_z)) * 0.5)
    span = abs(tip_z - root_z)

    _visual(
        part,
        Box((0.035, 0.035, span)),
        origin=Origin(xyz=(0.19, 0.055, center_z)),
        material=timber,
    )
    _visual(
        part,
        Box((0.035, 0.035, span)),
        origin=Origin(xyz=(-0.19, 0.055, center_z)),
        material=timber,
    )
    _visual(
        part,
        Box((0.42, 0.035, 0.04)),
        origin=Origin(xyz=(0.0, 0.055, root_z)),
        material=timber,
    )
    _visual(
        part,
        Box((0.42, 0.035, 0.04)),
        origin=Origin(xyz=(0.0, 0.055, tip_z)),
        material=timber,
    )

    for frac in (0.18, 0.37, 0.56, 0.75):
        slat_z = root_z + sign * (span * frac)
        _visual(
            part,
            Box((0.34, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.06, slat_z)),
            material=timber,
        )

    for x in (-0.11, 0.0, 0.11):
        _visual(
            part,
            Box((0.09, 0.006, span - 0.08)),
            origin=Origin(xyz=(x, 0.067, center_z)),
            material=canvas,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill", assets=ASSETS)
    mats = _materials()

    if hasattr(model, "materials"):
        for material in mats.values():
            if material is not None:
                model.materials.append(material)

    body = model.part("body")

    tower_profile = [
        (0.0, 0.0),
        (0.86, 0.0),
        (0.84, 0.10),
        (0.81, 0.45),
        (0.77, 0.95),
        (0.72, 1.55),
        (0.66, 2.12),
        (0.60, 2.48),
        (0.0, 2.48),
    ]
    tower_mesh = mesh_from_geometry(
        LatheGeometry(tower_profile, segments=56), ASSETS.mesh_path("tower.obj")
    )
    _visual(body, tower_mesh, material=mats["stone"])
    _visual(
        body,
        Cylinder(radius=0.84, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=mats["stone"],
    )
    _visual(
        body,
        Cylinder(radius=0.80, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=mats["stone"],
    )
    _visual(
        body,
        Cylinder(radius=0.73, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
        material=mats["stone"],
    )
    _visual(
        body,
        Cylinder(radius=0.66, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 2.18)),
        material=mats["stone"],
    )

    gallery_outer = superellipse_profile(1.84, 1.84, exponent=2.2, segments=48)
    gallery_inner = superellipse_profile(1.28, 1.28, exponent=2.0, segments=48)
    gallery_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(gallery_outer, [gallery_inner], height=0.065, center=True),
        ASSETS.mesh_path("gallery_deck.obj"),
    )
    rail_outer = superellipse_profile(1.92, 1.92, exponent=2.0, segments=48)
    rail_inner = superellipse_profile(1.74, 1.74, exponent=2.0, segments=48)
    rail_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(rail_outer, [rail_inner], height=0.04, center=True),
        ASSETS.mesh_path("gallery_rail.obj"),
    )
    _visual(body, gallery_mesh, origin=Origin(xyz=(0.0, 0.0, 2.18)), material=mats["timber_dark"])
    _visual(body, rail_mesh, origin=Origin(xyz=(0.0, 0.0, 2.34)), material=mats["timber_dark"])

    for angle in range(8):
        theta = angle * (math.pi / 4.0)
        x = 0.82 * math.cos(theta)
        y = 0.82 * math.sin(theta)
        _visual(
            body,
            Cylinder(radius=0.025, length=0.145),
            origin=Origin(xyz=(x, y, 2.285)),
            material=mats["timber_dark"],
        )

    cap_profile = [
        (0.0, 0.0),
        (0.74, 0.0),
        (0.71, 0.04),
        (0.58, 0.18),
        (0.37, 0.42),
        (0.16, 0.60),
        (0.0, 0.68),
    ]
    cap_mesh = mesh_from_geometry(
        LatheGeometry(cap_profile, segments=52), ASSETS.mesh_path("cap.obj")
    )
    _visual(body, cap_mesh, origin=Origin(xyz=(0.0, 0.0, 2.46)), material=mats["roof"])
    _visual(
        body,
        Cylinder(radius=0.75, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 2.48)),
        material=mats["timber_dark"],
    )

    _visual(
        body,
        Box((0.26, 0.24, 0.22)),
        origin=Origin(xyz=(0.0, 0.76, 2.70)),
        material=mats["timber_dark"],
    )
    _visual(
        body,
        Cylinder(radius=0.12, length=0.04),
        origin=Origin(xyz=(0.0, 0.953, 2.70), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["iron"],
    )
    _visual(
        body,
        Box((0.14, 0.18, 0.24)),
        origin=Origin(xyz=(0.18, 0.84, 2.70)),
        material=mats["timber_dark"],
    )
    _visual(
        body,
        Box((0.14, 0.18, 0.24)),
        origin=Origin(xyz=(-0.18, 0.84, 2.70)),
        material=mats["timber_dark"],
    )
    _visual(
        body,
        Box((0.48, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.86, 2.83)),
        material=mats["timber_dark"],
    )

    _visual(
        body,
        Box((0.08, 0.95, 0.08)),
        origin=Origin(xyz=(0.0, -0.78, 2.72)),
        material=mats["timber_dark"],
    )
    _visual(
        body,
        Box((0.55, 0.06, 0.42)),
        origin=Origin(xyz=(0.0, -1.22, 2.72)),
        material=mats["timber_dark"],
    )

    _visual(
        body,
        Box((0.46, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.75, 0.06)),
        material=mats["stone"],
    )
    _visual(
        body,
        Box((0.28, 0.06, 0.58)),
        origin=Origin(xyz=(0.0, 0.76, 0.31)),
        material=mats["timber"],
    )
    _visual(
        body,
        Box((0.18, 0.06, 0.22)),
        origin=Origin(xyz=(0.0, 0.64, 1.40)),
        material=mats["timber_dark"],
    )
    _visual(
        body,
        Box((0.06, 0.24, 0.26)),
        origin=Origin(xyz=(0.68, 0.02, 1.56)),
        material=mats["timber_dark"],
    )
    _visual(
        body,
        Box((0.06, 0.22, 0.22)),
        origin=Origin(xyz=(-0.64, -0.08, 1.92)),
        material=mats["timber_dark"],
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.84, length=3.15),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 1.575)),
    )

    sails = model.part("sails")
    _visual(
        sails,
        Cylinder(radius=0.10, length=0.20),
        origin=Origin(xyz=(0.0, 0.10, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["iron"],
    )
    _visual(
        sails,
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(xyz=(0.0, 0.09, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mats["timber_dark"],
    )
    _visual(
        sails,
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.0, 0.22, 0.0)),
        material=mats["iron"],
    )
    _visual(
        sails,
        Box((3.25, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.05, 0.0)),
        material=mats["timber_dark"],
    )
    _visual(
        sails,
        Box((0.12, 0.10, 3.25)),
        origin=Origin(xyz=(0.0, 0.05, 0.0)),
        material=mats["timber_dark"],
    )

    _add_horizontal_sail(sails, 1.0, mats["timber"], mats["canvas"])
    _add_horizontal_sail(sails, -1.0, mats["timber"], mats["canvas"])
    _add_vertical_sail(sails, 1.0, mats["timber"], mats["canvas"])
    _add_vertical_sail(sails, -1.0, mats["timber"], mats["canvas"])

    sails.inertial = Inertial.from_geometry(
        Box((3.30, 0.24, 3.30)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
    )

    model.articulation(
        "sail_spin",
        ArticulationType.CONTINUOUS,
        parent="body",
        child="sails",
        origin=Origin(xyz=(0.0, 0.982, 2.70)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=256, overlap_tol=0.003, overlap_volume_tol=0.0)

    body_pos = _extract_xyz(ctx.part_world_position("body"))
    sail_pos = _extract_xyz(ctx.part_world_position("sails"))
    body_aabb = _extract_aabb_bounds(ctx.part_world_aabb("body"))
    sail_aabb = _extract_aabb_bounds(ctx.part_world_aabb("sails"))

    body_width = body_aabb[1] - body_aabb[0]
    body_depth = body_aabb[3] - body_aabb[2]
    body_height = body_aabb[5] - body_aabb[4]
    sail_width = sail_aabb[1] - sail_aabb[0]
    sail_height = sail_aabb[5] - sail_aabb[4]

    if abs(sail_pos[0] - body_pos[0]) > 0.03:
        raise AssertionError("Rotor should stay centered laterally on the tower.")
    if sail_pos[1] <= body_pos[1] + 0.90:
        raise AssertionError("Rotor should project clearly in front of the mill body.")
    if sail_pos[2] <= body_pos[2] + 2.55:
        raise AssertionError("Rotor should be mounted high on the cap, not low on the tower.")
    if body_width <= 1.55:
        raise AssertionError("Windmill body should read as a substantial tower.")
    if body_depth <= 2.10:
        raise AssertionError("Windmill body should include meaningful front and rear structure.")
    if body_height <= 3.00:
        raise AssertionError("Windmill body should have a tall architectural presence.")
    if sail_width <= 3.10 or sail_height <= 3.10:
        raise AssertionError(
            "Sail assembly should span broadly and read as full-sized windmill sails."
        )
    if sail_aabb[2] < body_aabb[3] - 0.06:
        raise AssertionError("Sails should remain in front of the tower envelope.")
    if sail_aabb[4] <= 0.95:
        raise AssertionError("Lowest sail sweep should stay safely above the ground plane.")
    forward_gap = sail_aabb[2] - body_aabb[3]
    if forward_gap < 0.005 or forward_gap > 0.04:
        raise AssertionError(
            "Sails should mount just ahead of the windshaft housing with only a small clearance."
        )

    ctx.expect_origin_distance("sails", "body", axes="xy", max_dist=1.05)

    with ctx.pose(sail_spin=math.pi / 4.0):
        ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=1, overlap_tol=0.003, overlap_volume_tol=0.0)
        diag_aabb = _extract_aabb_bounds(ctx.part_world_aabb("sails"))
        diag_width = diag_aabb[1] - diag_aabb[0]
        diag_height = diag_aabb[5] - diag_aabb[4]
        if diag_aabb[2] < body_aabb[3] - 0.08:
            raise AssertionError(
                "Diagonal sail pose should still stay forward of the cap and tower."
            )
        if diag_aabb[4] <= 1.20:
            raise AssertionError(
                "Diagonal sail pose should keep the lower tip well clear of the ground."
            )
        if diag_width <= 2.20 or diag_height <= 2.20:
            raise AssertionError(
                "Diagonal sail pose should preserve a large readable rotating cross."
            )
        diag_gap = diag_aabb[2] - body_aabb[3]
        if diag_gap < 0.005 or diag_gap > 0.04:
            raise AssertionError(
                "Diagonal sail pose should keep the hub closely mounted ahead of the cap."
            )

    with ctx.pose(sail_spin=math.pi / 2.0):
        ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=1, overlap_tol=0.003, overlap_volume_tol=0.0)
        quarter_aabb = _extract_aabb_bounds(ctx.part_world_aabb("sails"))
        if (quarter_aabb[1] - quarter_aabb[0]) <= 3.10:
            raise AssertionError("Quarter-turn pose should preserve the full horizontal sail span.")
        if (quarter_aabb[5] - quarter_aabb[4]) <= 3.10:
            raise AssertionError("Quarter-turn pose should preserve the full vertical sail span.")
        if quarter_aabb[2] < body_aabb[3] - 0.06:
            raise AssertionError(
                "Quarter-turn sail pose should remain in front of the windmill body."
            )
        quarter_gap = quarter_aabb[2] - body_aabb[3]
        if quarter_gap < 0.005 or quarter_gap > 0.04:
            raise AssertionError(
                "Quarter-turn sail pose should keep the hub close to the windshaft housing."
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
