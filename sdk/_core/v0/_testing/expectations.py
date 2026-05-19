from __future__ import annotations

from typing import Optional, Sequence, Union

from .common import (
    ArticulationType,
    _aabb_axis_gap,
    _aabb_axis_overlap,
    _aabb_axis_separation,
    _aabb_center,
    _axes_label,
    _axis_index,
    _named_ref,
    _normalize_axes_spec,
    _normalize_axis_name,
    _normalize_direction_name,
    _point_distance_on_axes,
)


class TestContextExpectationMixin:
    def expect_origin_distance(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        min_dist: float = 0.0,
        max_dist: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_origin_distance({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)
        min_dist_f = float(min_dist)
        if min_dist_f < 0.0:
            return self._record(check_name, False, "min_dist must be >= 0")
        max_dist_f = None if max_dist is None else float(max_dist)
        if max_dist_f is not None and max_dist_f < min_dist_f:
            return self._record(check_name, False, "max_dist must be >= min_dist")

        pa = self.link_world_position(link_a)
        pb = self.link_world_position(link_b)
        if pa is None or pb is None:
            return self._record(check_name, False, "missing link world position(s)")

        dist = _point_distance_on_axes(pa, pb, axes=axes_key)
        ok = dist >= min_dist_f
        if max_dist_f is not None:
            ok = ok and dist <= max_dist_f
        upper_txt = "inf" if max_dist_f is None else f"{max_dist_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"origin_dist[{_axes_label(axes_key)}]({link_a_name!r},{link_b_name!r})={dist:.4g} "
            f"min_dist={min_dist_f:.4g} max_dist={upper_txt} pose={self._pose}",
        )

    def expect_origin_gap(
        self,
        positive_link: object,
        negative_link: object,
        *,
        axis: str,
        min_gap: float = 0.0,
        max_gap: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        positive_name = _named_ref(positive_link, kind="positive_link")
        negative_name = _named_ref(negative_link, kind="negative_link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(axis)
        check_name = (
            name or f"expect_origin_gap({positive_name},{negative_name},axis={axis_key or axis})"
        )
        if axis_err is not None or axis_key is None or axis_sign < 0.0:
            return self._record(check_name, False, axis_err or "axis must be one of: x, y, z")
        min_gap_f = float(min_gap)
        max_gap_f = None if max_gap is None else float(max_gap)
        if max_gap_f is not None and max_gap_f < min_gap_f:
            return self._record(check_name, False, "max_gap must be >= min_gap")

        positive_pos = self.link_world_position(positive_link)
        negative_pos = self.link_world_position(negative_link)
        if positive_pos is None or negative_pos is None:
            return self._record(check_name, False, "missing link world position(s)")

        idx = _axis_index(axis_key)
        gap = float(positive_pos[idx]) - float(negative_pos[idx])
        ok = gap >= min_gap_f
        if max_gap_f is not None:
            ok = ok and gap <= max_gap_f
        upper_txt = "inf" if max_gap_f is None else f"{max_gap_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"origin_gap_{axis_key}={gap:.4g} min_gap={min_gap_f:.4g} max_gap={upper_txt} pose={self._pose}",
        )

    def expect_contact(
        self,
        link_a: object,
        link_b: object,
        *,
        contact_tol: float = 1e-6,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        check_name = name or f"expect_contact({link_a_name},{link_b_name})"
        contact_tol_f = float(contact_tol)
        if contact_tol_f < 0.0:
            return self._record(check_name, False, "contact_tol must be >= 0")

        elements_a, _resolved_a, elem_a_name, error_a = self._resolve_exact_elements(
            link_a,
            elem=elem_a,
            kind_prefix="elem_a",
        )
        elements_b, _resolved_b, elem_b_name, error_b = self._resolve_exact_elements(
            link_b,
            elem=elem_b,
            kind_prefix="elem_b",
        )
        if error_a or error_b or elements_a is None or elements_b is None:
            errors = [item for item in (error_a, error_b) if item]
            return self._record(check_name, False, "; ".join(errors))

        min_distance, collided = self._exact_pair_distance(elements_a, elements_b)
        ok = collided or min_distance <= contact_tol_f
        return self._record(
            check_name,
            ok,
            f"min_distance={min_distance:.4g} contact_tol={contact_tol_f:.4g} "
            f"elem_a={elem_a_name!r} elem_b={elem_b_name!r} pose={self._pose}",
        )

    def expect_gap(
        self,
        positive_link: object,
        negative_link: object,
        *,
        axis: str,
        min_gap: Optional[float] = None,
        max_gap: Optional[float] = None,
        max_penetration: Optional[float] = None,
        positive_elem: Optional[object] = None,
        negative_elem: Optional[object] = None,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        positive_name = _named_ref(positive_link, kind="positive_link")
        negative_name = _named_ref(negative_link, kind="negative_link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(axis)
        check_name = name or f"expect_gap({positive_name},{negative_name},axis={axis_key or axis})"
        if axis_err is not None or axis_key is None or axis_sign < 0.0:
            return self._record(check_name, False, axis_err or "axis must be one of: x, y, z")
        if positive_elem is not None and elem_a is not None:
            raise TypeError("expect_gap() accepts only one of 'positive_elem' or alias 'elem_a'")
        if negative_elem is not None and elem_b is not None:
            raise TypeError("expect_gap() accepts only one of 'negative_elem' or alias 'elem_b'")
        if elem_a is not None:
            positive_elem = elem_a
        if elem_b is not None:
            negative_elem = elem_b

        positive_elements, _resolved_positive, positive_elem_name, positive_error = (
            self._resolve_exact_elements(
                positive_link,
                elem=positive_elem,
                kind_prefix="positive",
            )
        )
        negative_elements, _resolved_negative, negative_elem_name, negative_error = (
            self._resolve_exact_elements(
                negative_link,
                elem=negative_elem,
                kind_prefix="negative",
            )
        )
        if (
            positive_error
            or negative_error
            or positive_elements is None
            or negative_elements is None
        ):
            errors = [item for item in (positive_error, negative_error) if item]
            return self._record(check_name, False, "; ".join(errors))

        positive_min, _positive_max = self._elements_projection_interval(
            positive_elements,
            axis=axis_key,
        )
        _negative_min, negative_max = self._elements_projection_interval(
            negative_elements,
            axis=axis_key,
        )

        if min_gap is None:
            max_penetration_f = 0.0 if max_penetration is None else float(max_penetration)
            min_gap_f = -max_penetration_f
        else:
            min_gap_f = float(min_gap)
            if max_penetration is not None:
                max_penetration_f = float(max_penetration)
                expected_min_gap = -max_penetration_f
                if abs(expected_min_gap - min_gap_f) > 1e-9:
                    self.warn(
                        "expect_gap received both min_gap and max_penetration with different bounds; "
                        "using min_gap as the lower bound."
                    )
            else:
                max_penetration_f = max(0.0, -min_gap_f)
        max_gap_f = None if max_gap is None else float(max_gap)
        if max_gap_f is not None and max_gap_f < min_gap_f:
            return self._record(check_name, False, "max_gap must be >= min_gap")

        gap = positive_min - negative_max
        ok = gap >= min_gap_f
        if max_gap_f is not None:
            ok = ok and gap <= max_gap_f
        upper_txt = "inf" if max_gap_f is None else f"{max_gap_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"gap_{axis_key}={gap:.4g} min_gap={min_gap_f:.4g} max_gap={upper_txt} "
            f"max_penetration={max_penetration_f:.4g} "
            f"positive_elem={positive_elem_name!r} negative_elem={negative_elem_name!r} "
            f"pose={self._pose}",
        )

    def expect_overlap(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        min_overlap: float = 0.0,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_overlap({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        elements_a, _resolved_a, elem_a_name, error_a = self._resolve_exact_elements(
            link_a,
            elem=elem_a,
            kind_prefix="elem_a",
        )
        elements_b, _resolved_b, elem_b_name, error_b = self._resolve_exact_elements(
            link_b,
            elem=elem_b,
            kind_prefix="elem_b",
        )
        if error_a or error_b or elements_a is None or elements_b is None:
            errors = [item for item in (error_a, error_b) if item]
            return self._record(check_name, False, "; ".join(errors))

        min_overlap_f = float(min_overlap)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            min_a, max_a = self._elements_projection_interval(elements_a, axis=axis_name)
            min_b, max_b = self._elements_projection_interval(elements_b, axis=axis_name)
            overlap = min(max_a, max_b) - max(min_a, min_b)
            axis_ok = overlap >= min_overlap_f
            ok = ok and axis_ok
            axis_details.append(f"overlap_{axis_name}={overlap:.4g}")

        return self._record(
            check_name,
            ok,
            " ".join(axis_details)
            + f" min_overlap={min_overlap_f:.4g} elem_a={elem_a_name!r} elem_b={elem_b_name!r} pose={self._pose}",
        )

    def expect_within(
        self,
        inner_link: object,
        outer_link: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        margin: float = 0.0,
        inner_elem: Optional[object] = None,
        outer_elem: Optional[object] = None,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        inner_name = _named_ref(inner_link, kind="inner_link")
        outer_name = _named_ref(outer_link, kind="outer_link")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name or f"expect_within({inner_name},{outer_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)
        if inner_elem is not None and elem_a is not None:
            raise TypeError("expect_within() accepts only one of 'inner_elem' or alias 'elem_a'")
        if outer_elem is not None and elem_b is not None:
            raise TypeError("expect_within() accepts only one of 'outer_elem' or alias 'elem_b'")
        if elem_a is not None:
            inner_elem = elem_a
        if elem_b is not None:
            outer_elem = elem_b

        inner_elements, _resolved_inner, inner_elem_name, inner_error = (
            self._resolve_exact_elements(
                inner_link,
                elem=inner_elem,
                kind_prefix="inner",
            )
        )
        outer_elements, _resolved_outer, outer_elem_name, outer_error = (
            self._resolve_exact_elements(
                outer_link,
                elem=outer_elem,
                kind_prefix="outer",
            )
        )
        if inner_error or outer_error or inner_elements is None or outer_elements is None:
            errors = [item for item in (inner_error, outer_error) if item]
            return self._record(check_name, False, "; ".join(errors))

        margin_f = float(margin)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            inner_min, inner_max = self._elements_projection_interval(
                inner_elements, axis=axis_name
            )
            outer_min, outer_max = self._elements_projection_interval(
                outer_elements, axis=axis_name
            )
            axis_ok = inner_min >= outer_min - margin_f and inner_max <= outer_max + margin_f
            ok = ok and axis_ok
            axis_details.append(
                f"{axis_name}=({inner_min:.4g},{inner_max:.4g}) in ({outer_min:.4g},{outer_max:.4g})"
            )

        return self._record(
            check_name,
            ok,
            f"inner={inner_name!r} outer={outer_name!r} axes={_axes_label(axes_key)} "
            f"margin={margin_f:.4g} inner_elem={inner_elem_name!r} outer_elem={outer_elem_name!r} "
            + " ".join(axis_details)
            + f" pose={self._pose}",
        )

    def expect_aabb_within(
        self,
        inner_link: object,
        outer_link: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        margin: float = 0.0,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_within(...)", "expect_within(...)")
        inner_name = _named_ref(inner_link, kind="inner_link")
        outer_name = _named_ref(outer_link, kind="outer_link")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_within({inner_name},{outer_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        inner = self.link_world_aabb(inner_link)
        outer = self.link_world_aabb(outer_link)
        if inner is None or outer is None:
            return self._record(check_name, False, "missing link AABB(s)")

        margin_f = float(margin)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            idx = _axis_index(axis_name)
            inner_min = float(inner[0][idx])
            inner_max = float(inner[1][idx])
            outer_min = float(outer[0][idx])
            outer_max = float(outer[1][idx])
            axis_ok = inner_min >= outer_min - margin_f and inner_max <= outer_max + margin_f
            ok = ok and axis_ok
            axis_details.append(
                f"{axis_name}=({inner_min:.4g},{inner_max:.4g}) in ({outer_min:.4g},{outer_max:.4g})"
            )

        return self._record(
            check_name,
            ok,
            f"inner={inner_name!r} outer={outer_name!r} axes={_axes_label(axes_key)} margin={margin_f:.4g} "
            + " ".join(axis_details)
            + f" pose={self._pose}",
        )

    def expect_aabb_gap(
        self,
        positive_link: object,
        negative_link: object,
        *,
        axis: str,
        min_gap: Optional[float] = None,
        max_gap: Optional[float] = None,
        max_penetration: Optional[float] = None,
        positive_elem: Optional[object] = None,
        negative_elem: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_gap(...)", "expect_gap(...)")
        """
        Check the signed directional gap between two links or two named elements.

        The measured gap is:

            positive.min[axis] - negative.max[axis]

        along the requested positive world axis. This makes the check directional:
        `expect_aabb_gap("lid", "base", axis="z")` means "how far is the bottom
        of the lid above the top of the base?", while swapping the arguments asks
        a different question and flips the sign convention.

        Parameters:
          - positive_link / negative_link: the objects on the positive-side and
            negative-side of the tested axis
          - axis: one of "x", "y", or "z"; must be the positive axis direction
          - min_gap: lower bound on the signed gap; when omitted, derive it from
            `max_penetration`
          - max_gap: upper bound on the signed gap
          - max_penetration: convenience way to express the allowed overlap depth;
            this becomes a lower bound of `-max_penetration`
          - positive_elem / negative_elem: optional named geometry items within
            each link; when provided, the check uses those element AABBs instead
            of the whole-link union AABBs
          - name: optional explicit check name for reporting
        """
        positive_name = _named_ref(positive_link, kind="positive_link")
        negative_name = _named_ref(negative_link, kind="negative_link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(axis)
        check_name = (
            name or f"expect_aabb_gap({positive_name},{negative_name},axis={axis_key or axis})"
        )
        if axis_err is not None or axis_key is None or axis_sign < 0.0:
            return self._record(check_name, False, axis_err or "axis must be one of: x, y, z")

        positive_elem_name = (
            None if positive_elem is None else _named_ref(positive_elem, kind="positive_elem")
        )
        negative_elem_name = (
            None if negative_elem is None else _named_ref(negative_elem, kind="negative_elem")
        )

        positive_aabb = (
            self.link_world_aabb(positive_link)
            if positive_elem_name is None
            else self.part_element_world_aabb(
                positive_link,
                elem=positive_elem_name,
            )
        )
        negative_aabb = (
            self.link_world_aabb(negative_link)
            if negative_elem_name is None
            else self.part_element_world_aabb(
                negative_link,
                elem=negative_elem_name,
            )
        )
        if positive_aabb is None or negative_aabb is None:
            missing: list[str] = []
            if positive_aabb is None:
                if positive_elem_name is None:
                    missing.append(f"missing link AABB for {positive_name!r}")
                else:
                    missing.append(
                        f"missing element AABB for positive_elem={positive_elem_name!r} on {positive_name!r}"
                    )
            if negative_aabb is None:
                if negative_elem_name is None:
                    missing.append(f"missing link AABB for {negative_name!r}")
                else:
                    missing.append(
                        f"missing element AABB for negative_elem={negative_elem_name!r} on {negative_name!r}"
                    )
            return self._record(check_name, False, "; ".join(missing))

        if min_gap is None:
            max_penetration_f = 0.0 if max_penetration is None else float(max_penetration)
            min_gap_f = -max_penetration_f
        else:
            min_gap_f = float(min_gap)
            if max_penetration is not None:
                max_penetration_f = float(max_penetration)
                expected_min_gap = -max_penetration_f
                if abs(expected_min_gap - min_gap_f) > 1e-9:
                    self.warn(
                        "expect_aabb_gap received both min_gap and max_penetration with different bounds; "
                        "using min_gap as the lower bound."
                    )
            else:
                max_penetration_f = max(0.0, -min_gap_f)
        max_gap_f = None if max_gap is None else float(max_gap)
        if max_gap_f is not None and max_gap_f < min_gap_f:
            return self._record(check_name, False, "max_gap must be >= min_gap")

        gap = _aabb_axis_gap(positive_aabb, negative_aabb, axis=axis_key)
        ok = gap >= min_gap_f
        if max_gap_f is not None:
            ok = ok and gap <= max_gap_f
        upper_txt = "inf" if max_gap_f is None else f"{max_gap_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"gap_{axis_key}={gap:.4g} min_gap={min_gap_f:.4g} max_gap={upper_txt} "
            f"max_penetration={max_penetration_f:.4g} "
            f"positive_elem={positive_elem_name!r} negative_elem={negative_elem_name!r} "
            f"pose={self._pose}",
        )

    def expect_aabb_overlap(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        min_overlap: float = 0.0,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_overlap(...)", "expect_overlap(...)")
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_overlap({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        aabb_a = self.link_world_aabb(link_a)
        aabb_b = self.link_world_aabb(link_b)
        if aabb_a is None or aabb_b is None:
            return self._record(check_name, False, "missing link AABB(s)")

        min_overlap_f = float(min_overlap)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            overlap = _aabb_axis_overlap(aabb_a, aabb_b, axis=axis_name)
            axis_ok = overlap >= min_overlap_f
            ok = ok and axis_ok
            axis_details.append(f"overlap_{axis_name}={overlap:.4g}")

        return self._record(
            check_name,
            ok,
            " ".join(axis_details) + f" min_overlap={min_overlap_f:.4g} pose={self._pose}",
        )

    def expect_aabb_contact(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xyz",
        contact_tol: float = 0.0,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_contact(...)", "expect_contact(...)")
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_contact({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        contact_tol_f = float(contact_tol)
        if contact_tol_f < 0.0:
            return self._record(check_name, False, "contact_tol must be >= 0")

        aabb_a = self.link_world_aabb(link_a)
        aabb_b = self.link_world_aabb(link_b)
        if aabb_a is None or aabb_b is None:
            return self._record(check_name, False, "missing link AABB(s)")

        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            sep = _aabb_axis_separation(aabb_a, aabb_b, axis=axis_name)
            axis_ok = sep <= contact_tol_f
            ok = ok and axis_ok
            axis_details.append(f"sep_{axis_name}={sep:.4g}")

        return self._record(
            check_name,
            ok,
            " ".join(axis_details) + f" contact_tol={contact_tol_f:.4g} pose={self._pose}",
        )

    def expect_joint_motion_axis(
        self,
        joint: object,
        link: object,
        *,
        world_axis: Union[str, Sequence[float]],
        direction: Union[str, float, int],
        min_delta: float = 0.0,
        q0: Optional[float] = None,
        q1: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper(
            "expect_joint_motion_axis(...)",
            "pose-specific exact contact/gap/overlap checks",
        )
        """
        Check that moving one joint from q0 -> q1 moves a link center along a world axis
        in the expected sign direction.

        This is useful for catching hinge/pivot sign mistakes, such as a trash-can lid
        rotating inward instead of outward.

        Parameters:
          - world_axis: one of "x", "y", "z"
          - direction: "positive" or "negative"
          - min_delta: minimum magnitude along the chosen axis in the requested direction
          - q0/q1: optional explicit samples; when omitted, derive from joint limits
                   (continuous joints default to 0 -> pi/2)
        """

        j_name = _named_ref(joint, kind="joint")
        l_name = _named_ref(link, kind="link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(world_axis)
        if axis_err is not None or axis_key is None:
            return self._record(
                name or f"expect_joint_motion_axis({j_name},{l_name},{world_axis},{direction})",
                False,
                axis_err or "world_axis must be one of: x, y, z",
            )
        dir_key, dir_err = _normalize_direction_name(direction)
        if dir_err is not None or dir_key is None:
            return self._record(
                name or f"expect_joint_motion_axis({j_name},{l_name},{axis_key},{direction})",
                False,
                dir_err or "direction must be either 'positive' or 'negative'",
            )
        if axis_sign < 0.0:
            dir_key = "negative" if dir_key == "positive" else "positive"
        check_name = name or f"expect_joint_motion_axis({j_name},{l_name},{axis_key},{dir_key})"

        axis_index = {"x": 0, "y": 1, "z": 2}[axis_key]

        min_delta_f = float(min_delta)
        if min_delta_f < 0.0:
            return self._record(check_name, False, "min_delta must be >= 0")

        joint_obj = None
        get_joint = getattr(self.model, "get_articulation", None)
        if not callable(get_joint):
            get_joint = getattr(self.model, "get_joint", None)
        if callable(get_joint):
            try:
                joint_obj = get_joint(j_name)
            except Exception:
                joint_obj = None
        if joint_obj is None:
            joints = getattr(self.model, "articulations", None)
            if not isinstance(joints, list):
                joints = getattr(self.model, "joints", None)
            if isinstance(joints, list):
                for j in joints:
                    if getattr(j, "name", None) == j_name:
                        joint_obj = j
                        break
        if joint_obj is None:
            return self._record(check_name, False, f"joint {j_name!r} not found")
        mimic = getattr(joint_obj, "mimic", None)
        if mimic is not None:
            source_name = getattr(mimic, "joint", None)
            return self._record(
                check_name,
                False,
                f"joint {j_name!r} is mimic-driven; drive source joint {source_name!r} instead",
            )

        if q0 is None or q1 is None:
            jt = getattr(joint_obj, "joint_type", None)
            lim = getattr(joint_obj, "limit", None)
            if jt == ArticulationType.CONTINUOUS:
                default_q0 = 0.0
                default_q1 = 1.5707963267948966  # pi/2
            else:
                lower = None if lim is None else getattr(lim, "lower", None)
                upper = None if lim is None else getattr(lim, "upper", None)
                if lower is None or upper is None:
                    return self._record(
                        check_name,
                        False,
                        f"joint {j_name!r} has no finite lower/upper limits; provide q0 and q1 explicitly",
                    )
                default_q0 = float(lower)
                default_q1 = float(upper)
        else:
            default_q0 = 0.0
            default_q1 = 0.0

        q0_f = float(default_q0 if q0 is None else q0)
        q1_f = float(default_q1 if q1 is None else q1)
        if abs(q1_f - q0_f) <= 1e-12:
            return self._record(
                check_name, False, f"q0 and q1 must differ (q0={q0_f:.6g}, q1={q1_f:.6g})"
            )

        base_pose = dict(self._pose)
        pose0 = dict(base_pose)
        pose1 = dict(base_pose)
        pose0[j_name] = q0_f
        pose1[j_name] = q1_f

        with self.pose(pose0):
            aabb0 = self.link_world_aabb(l_name)
        with self.pose(pose1):
            aabb1 = self.link_world_aabb(l_name)
        if aabb0 is None or aabb1 is None:
            return self._record(check_name, False, f"missing AABB for link {l_name!r}")

        c0 = _aabb_center(aabb0)
        c1 = _aabb_center(aabb1)
        delta = float(c1[axis_index]) - float(c0[axis_index])

        if dir_key == "positive":
            ok = delta >= min_delta_f
            expect_txt = f"delta_{axis_key} >= {min_delta_f:.4g}"
        else:
            ok = delta <= -min_delta_f
            expect_txt = f"delta_{axis_key} <= {-min_delta_f:.4g}"

        details = (
            f"{expect_txt}; got delta_{axis_key}={delta:.4g} "
            f"for joint={j_name!r} link={l_name!r} q0={q0_f:.4g} q1={q1_f:.4g} "
            f"center0=({c0[0]:.4g},{c0[1]:.4g},{c0[2]:.4g}) "
            f"center1=({c1[0]:.4g},{c1[1]:.4g},{c1[2]:.4g}) pose_base={base_pose}"
        )
        if not ok and getattr(joint_obj, "joint_type", None) == ArticulationType.REVOLUTE:
            axis = getattr(joint_obj, "axis", None)
            details += (
                f" | Right-hand rule hint: positive rotation follows the right-hand rule around axis={axis}. "
                "If motion is reversed, flip the joint axis sign."
            )

        return self._record(check_name, ok, details)
