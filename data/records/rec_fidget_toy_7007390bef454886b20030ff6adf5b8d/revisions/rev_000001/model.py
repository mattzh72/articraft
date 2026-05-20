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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_THICKNESS = 0.0065
CAP_THICKNESS = 0.0018
CAP_CLEARANCE = 0.00030
BEARING_OUTER_RADIUS = 0.0112


def _circle_profile(radius: float, *, samples: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / samples),
            radius * math.sin((2.0 * math.pi * index) / samples),
        )
        for index in range(samples)
    ]


def _rounded_rect_sdf(
    x: float,
    y: float,
    *,
    center: tuple[float, float],
    size: tuple[float, float],
    radius: float,
) -> float:
    cx, cy = center
    sx, sy = size
    qx = abs(x - cx) - sx * 0.5 + radius
    qy = abs(y - cy) - sy * 0.5 + radius
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    inside = min(max(qx, qy), 0.0)
    return outside + inside - radius


def _disk_sdf(x: float, y: float, *, radius: float) -> float:
    return math.hypot(x, y) - radius


def _spinner_union_sdf(x: float, y: float) -> float:
    components = [
        _disk_sdf(x, y, radius=0.0168),
        _rounded_rect_sdf(
            x,
            y,
            center=(0.019, 0.0),
            size=(0.026, 0.013),
            radius=0.003,
        ),
        _rounded_rect_sdf(
            x,
            y,
            center=(-0.019, 0.0),
            size=(0.026, 0.013),
            radius=0.003,
        ),
        _rounded_rect_sdf(
            x,
            y,
            center=(0.0, 0.019),
            size=(0.013, 0.026),
            radius=0.003,
        ),
        _rounded_rect_sdf(
            x,
            y,
            center=(0.0, -0.019),
            size=(0.013, 0.026),
            radius=0.003,
        ),
        _rounded_rect_sdf(
            x,
            y,
            center=(0.032, 0.0),
            size=(0.028, 0.023),
            radius=0.006,
        ),
        _rounded_rect_sdf(
            x,
            y,
            center=(-0.032, 0.0),
            size=(0.028, 0.023),
            radius=0.006,
        ),
        _rounded_rect_sdf(
            x,
            y,
            center=(0.0, 0.032),
            size=(0.023, 0.028),
            radius=0.006,
        ),
        _rounded_rect_sdf(
            x,
            y,
            center=(0.0, -0.032),
            size=(0.023, 0.028),
            radius=0.006,
        ),
    ]
    return min(components)


def _spinner_outer_profile(*, samples: int = 180) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    max_radius = 0.06
    for index in range(samples):
        angle = (2.0 * math.pi * index) / samples
        dx = math.cos(angle)
        dy = math.sin(angle)
        lo = 0.0
        hi = max_radius
        while _spinner_union_sdf(dx * hi, dy * hi) <= 0.0 and hi < 0.20:
            hi *= 1.25
        for _ in range(28):
            mid = 0.5 * (lo + hi)
            if _spinner_union_sdf(dx * mid, dy * mid) <= 0.0:
                lo = mid
            else:
                hi = mid
        profile.append((dx * lo, dy * lo))
    return profile


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[index] + hi[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quad_lobe_fidget_spinner")

    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.46, 0.49, 0.52, 1.0))

    cap_top = model.part("cap_top")
    cap_z = BODY_THICKNESS * 0.5 + CAP_CLEARANCE + CAP_THICKNESS * 0.5
    cap_top.visual(
        Cylinder(radius=0.0098, length=CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, cap_z)),
        material=steel,
        name="top_cap",
    )
    cap_top.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0098, length=CAP_THICKNESS),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, cap_z)),
    )

    cap_bottom = model.part("cap_bottom")
    cap_bottom.visual(
        Cylinder(radius=0.0098, length=CAP_THICKNESS),
        material=steel,
        name="bottom_cap",
    )
    cap_bottom.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0098, length=CAP_THICKNESS),
        mass=0.008,
    )

    spinner = model.part("spinner")
    body_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _spinner_outer_profile(),
            [],
            BODY_THICKNESS,
            center=True,
        ),
        "spinner_body",
    )
    spinner.visual(body_mesh, material=body_black, name="body")
    spinner.visual(
        Cylinder(radius=BEARING_OUTER_RADIUS, length=0.0071),
        material=steel,
        name="bearing_hub",
    )

    weight_centers = (
        (0.033, 0.0, 0.0),
        (0.0, 0.033, 0.0),
        (-0.033, 0.0, 0.0),
        (0.0, -0.033, 0.0),
    )
    for index, (x, y, z) in enumerate(weight_centers):
        spinner.visual(
            Cylinder(radius=0.0065, length=0.0074),
            origin=Origin(xyz=(x, y, z)),
            material=dark_steel,
            name=f"weight_{index}",
        )
    spinner.inertial = Inertial.from_geometry(
        Box((0.092, 0.092, 0.008)),
        mass=0.060,
    )

    model.articulation(
        "cap_to_bottom",
        ArticulationType.FIXED,
        parent=cap_top,
        child=cap_bottom,
        origin=Origin(xyz=(0.0, 0.0, -cap_z)),
    )
    model.articulation(
        "cap_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=cap_top,
        child=spinner,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=60.0),
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

    # For bounded REVOLUTE/PRISMATIC joints, add exact lower/upper motion-limit
    # checks for prompt-critical contacts, clearances, and motion direction. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.expect_contact(lid, body, elem_a="hinge_leaf", elem_b="body_leaf")
    cap_top = object_model.get_part("cap_top")
    cap_bottom = object_model.get_part("cap_bottom")
    spinner = object_model.get_part("spinner")
    joint = object_model.get_articulation("cap_to_spinner")

    ctx.expect_contact(
        cap_top,
        spinner,
        elem_a="top_cap",
        elem_b="bearing_hub",
        contact_tol=1e-6,
        name="top cap supports the spinner at the hub",
    )
    ctx.expect_contact(
        spinner,
        cap_bottom,
        elem_a="bearing_hub",
        elem_b="bottom_cap",
        contact_tol=1e-6,
        name="bottom cap supports the spinner at the hub",
    )
    ctx.expect_overlap(
        cap_top,
        spinner,
        axes="xy",
        elem_a="top_cap",
        elem_b="bearing_hub",
        min_overlap=0.014,
        name="top cap stays centered over the bearing hub",
    )

    rest_center = _aabb_center(ctx.part_element_world_aabb(spinner, elem="weight_0"))
    with ctx.pose({joint: math.pi / 4.0}):
        turned_center = _aabb_center(ctx.part_element_world_aabb(spinner, elem="weight_0"))

    radial_ok = False
    moved_ok = False
    if rest_center is not None and turned_center is not None:
        rest_radius = math.hypot(rest_center[0], rest_center[1])
        turned_radius = math.hypot(turned_center[0], turned_center[1])
        radial_ok = abs(rest_radius - turned_radius) < 0.001
        moved_ok = (
            abs(rest_center[0] - turned_center[0]) > 0.009
            and abs(rest_center[1] - turned_center[1]) > 0.009
        )
    ctx.check(
        "spinner rotates one weight around the hub axis",
        radial_ok and moved_ok,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
