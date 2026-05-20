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
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_THICKNESS = 0.0066
RING_THICKNESS = 0.0082
HUB_RACE_RADIUS = 0.0042
HUB_RACE_LENGTH = 0.0080
BUTTON_RADIUS = 0.0135
BUTTON_LENGTH = 0.0028
BODY_PHASE = math.pi / 2.0


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        try:
            return Material(name=name, rgba=rgba)
        except TypeError:
            try:
                return Material(name, rgba)
            except TypeError:
                return Material(name=name)


def _circle_profile(
    radius: float,
    segments: int = 64,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / segments),
            cy + radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _spinner_outline(samples: int = 192) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for i in range(samples):
        theta = (2.0 * math.pi * i) / samples
        phase_theta = theta - BODY_PHASE
        radius = (
            0.0250 + 0.0095 * math.cos(3.0 * phase_theta) + 0.0015 * math.cos(6.0 * phase_theta)
        )
        points.append((radius * math.cos(theta), radius * math.sin(theta)))
    return points


def _lobe_centers(radius: float = 0.0260) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(BODY_PHASE + (2.0 * math.pi * i) / 3.0),
            radius * math.sin(BODY_PHASE + (2.0 * math.pi * i) / 3.0),
        )
        for i in range(3)
    ]


def _ring_mesh_filename(name: str, outer_radius: float, inner_radius: float, thickness: float):
    geom = ExtrudeWithHolesGeometry(
        outer_profile=_circle_profile(outer_radius, segments=80),
        hole_profiles=[_circle_profile(inner_radius, segments=56)],
        height=thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tri_lobe_fidget_spinner", assets=ASSETS)

    anodized_body = _material("anodized_body", (0.16, 0.22, 0.31, 1.0))
    brushed_steel = _material("brushed_steel", (0.73, 0.75, 0.78, 1.0))
    matte_black = _material("matte_black", (0.10, 0.10, 0.11, 1.0))
    rubber_black = _material("rubber_black", (0.18, 0.18, 0.19, 1.0))

    lobe_centers = _lobe_centers()

    body_shell = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile=_spinner_outline(),
            hole_profiles=[
                _circle_profile(0.0103, segments=72),
                *[_circle_profile(0.0087, segments=56, center=center) for center in lobe_centers],
            ],
            height=BODY_THICKNESS,
            cap=True,
            center=True,
            closed=True,
        ),
        ASSETS.mesh_path("spinner_body_shell.obj"),
    )
    center_bearing_ring = _ring_mesh_filename(
        "spinner_center_bearing_ring.obj",
        outer_radius=0.0112,
        inner_radius=0.0056,
        thickness=RING_THICKNESS,
    )
    lobe_weight_ring = _ring_mesh_filename(
        "spinner_lobe_weight_ring.obj",
        outer_radius=0.0098,
        inner_radius=0.0062,
        thickness=RING_THICKNESS,
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=HUB_RACE_RADIUS, length=HUB_RACE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="inner_race",
    )
    for sign in (-1.0, 1.0):
        hub.visual(
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, sign * 0.0054)),
            material=matte_black,
            name=f"button_base_{'bottom' if sign < 0.0 else 'top'}",
        )
        hub.visual(
            Cylinder(radius=0.0105, length=0.0012),
            origin=Origin(xyz=(0.0, 0.0, sign * 0.0062)),
            material=matte_black,
            name=f"button_shoulder_{'bottom' if sign < 0.0 else 'top'}",
        )
        hub.visual(
            Cylinder(radius=0.0055, length=0.0008),
            origin=Origin(xyz=(0.0, 0.0, sign * 0.0068)),
            material=brushed_steel,
            name=f"button_accent_{'bottom' if sign < 0.0 else 'top'}",
        )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=BUTTON_RADIUS, length=0.0144),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    spinner = model.part("spinner_frame")
    spinner.visual(
        body_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=anodized_body,
        name="body_shell",
    )
    spinner.visual(
        center_bearing_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="center_bearing_outer_race",
    )
    for index, (x_pos, y_pos) in enumerate(lobe_centers):
        spinner.visual(
            lobe_weight_ring,
            origin=Origin(xyz=(x_pos, y_pos, 0.0)),
            material=brushed_steel,
            name=f"weight_ring_{index}",
        )
        for sign in (-1.0, 1.0):
            spinner.visual(
                Cylinder(radius=0.0066, length=0.0008),
                origin=Origin(xyz=(x_pos, y_pos, sign * 0.0037)),
                material=rubber_black,
                name=f"grip_pad_{index}_{'bottom' if sign < 0.0 else 'top'}",
            )
    spinner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.010),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "hub_to_spinner",
        ArticulationType.CONTINUOUS,
        parent="hub",
        child="spinner_frame",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    for angle in (0.0, math.pi / 6.0, math.pi / 3.0, math.pi / 2.0, 2.05):
        with ctx.pose(hub_to_spinner=angle):
            ctx.expect_origin_distance("spinner_frame", "hub", axes="xy", max_dist=0.001)
            ctx.expect_aabb_overlap("spinner_frame", "hub", axes="xy", min_overlap=0.020)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
